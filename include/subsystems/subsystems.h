#pragma once

#include "bottomIntake.h"
#include "command/commandController.h"
#include "command/commandScheduler.h"
#include "command/conditionalCommand.h"
#include "command/parallelCommandGroup.h"
#include "command/parallelRaceGroup.h"
#include "command/proxyCommand.h"
#include "command/repeatCommand.h"
#include "command/scheduleCommand.h"
#include "command/sequence.h"
#include "command/waitCommand.h"
#include "command/waitUntilCommand.h"
#include "commands/driveMove.h"
#include "commands/intake/positionCommand.h"
#include "commands/intake/trapTopPosition.h"
#include "commands/ramsete.h"
#include "commands/rotate.h"
#include "drivetrain.h"
#include "solenoidSubsystem.h"
#include "lift.h"
#include "localization/distance.h"
#include "localization/gps.h"
#include "localization/line.h"
#include "motionProfiling/pathCommands.h"
#include "pros/adi.hpp"
#include "topIntake.h"

#include <queue>

inline DrivetrainSubsystem *drivetrainSubsystem;
inline TopIntakeSubsystem *topIntakeSubsystem;
inline MotorSubsystem *bottomIntakeSubsystem;
inline LiftSubsystem *liftSubsystem;
inline SolenoidSubsystem *goalClampSubsystem;
inline SolenoidSubsystem *hangSubsystem;

inline Command *loadLB;
inline Command *intakeOntoGoal;
inline Command *goalClampTrue;
inline Command *hang;
inline Command *barToBarHang;

inline CommandController primary(pros::controller_id_e_t::E_CONTROLLER_MASTER);
inline CommandController partner(pros::controller_id_e_t::E_CONTROLLER_PARTNER);

std::vector<int> ejectionPoints{};
RingColor lastColor;

inline void subsystemInit() {
    TELEMETRY.setSerial(new pros::Serial(6, 921600));

    topIntakeSubsystem = new TopIntakeSubsystem({-12, 13}, pros::Distance(20));
    bottomIntakeSubsystem = new MotorSubsystem(pros::Motor(18));
    liftSubsystem = new LiftSubsystem({-5, 7}, PID(4.5, 0.0, 3.0));
    goalClampSubsystem = new SolenoidSubsystem(pros::adi::DigitalOut('b'));
    drivetrainSubsystem = new DrivetrainSubsystem({-8, -16}, {3, 4}, {21}, {-2}, pros::Imu(14),
                                                  pros::adi::DigitalOut('a'),
                                                  pros::adi::DigitalOut('c'), []() {
                                                      return goalClampSubsystem->getLastValue();
                                                  });

    drivetrainSubsystem->addLocalizationSensor(new Distance(CONFIG::DISTANCE_LEFT_OFFSET, 2438.0 / 2485.0,
                                                            pros::Distance(15)));
    drivetrainSubsystem->addLocalizationSensor(new Distance(CONFIG::DISTANCE_FRONT_OFFSET, 2438.0 / 2480.0,
                                                            pros::Distance(1)));
    drivetrainSubsystem->addLocalizationSensor(new Distance(CONFIG::DISTANCE_RIGHT_OFFSET, 2438.0 / 2505.0,
                                                            pros::Distance(11)));
    drivetrainSubsystem->addLocalizationSensor(new Distance(CONFIG::DISTANCE_BACK_OFFSET, 2438.0 / 2483.0,
                                                            pros::Distance(9)));

    drivetrainSubsystem->initUniform(-70_in, -70_in, 70_in, 70_in, 0_deg, false);

    CommandScheduler::registerSubsystem(drivetrainSubsystem, drivetrainSubsystem->tank(primary));
    CommandScheduler::registerSubsystem(
        topIntakeSubsystem, topIntakeSubsystem->pctCommand(0.0));
    CommandScheduler::registerSubsystem(bottomIntakeSubsystem, bottomIntakeSubsystem->stopIntake());
    CommandScheduler::registerSubsystem(liftSubsystem, liftSubsystem->positionCommand(0.0_deg));
    CommandScheduler::registerSubsystem(goalClampSubsystem, goalClampSubsystem->levelCommand(false));
    CommandScheduler::registerSubsystem(hangSubsystem, hangSubsystem->levelCommand(false));

    goalClampTrue = goalClampSubsystem->levelCommand(true);

    intakeOntoGoal = new ParallelCommandGroup({
        bottomIntakeSubsystem->pctCommand(1.0), topIntakeSubsystem->pctCommand(1.0)
    });
    loadLB = new ParallelRaceGroup({
        bottomIntakeSubsystem->pctCommand(1.0), topIntakeSubsystem->pctCommand(1.0),
        liftSubsystem->positionCommand(CONFIG::WALL_STAKE_LOAD_HEIGHT)
    }); // Add stall detection code

    barToBarHang =
            new Sequence({
                liftSubsystem->positionCommand(35_deg)->race(drivetrainSubsystem->hangUp(1.0, 8.32_in)),
                liftSubsystem->positionCommand(72_deg)->race(drivetrainSubsystem->hangPctCommand(0.0))->withTimeout(
                    0.35_s),
                liftSubsystem->positionCommand(78_deg)->race(drivetrainSubsystem->hangDown(-1.0, 4_in)),
                liftSubsystem->positionCommand(100_deg)->race(drivetrainSubsystem->hangDown(-1.0, -2.5_in)),
                liftSubsystem->positionCommand(25_deg)->race(drivetrainSubsystem->hangPctCommand(-0.57))->
                withTimeout(0.3_s),
                liftSubsystem->positionCommand(90_deg)->race(drivetrainSubsystem->hangPctCommand(1.0))->withTimeout(
                    0.1_s)
            });
    hang = new Sequence({
        drivetrainSubsystem->activatePto(), drivetrainSubsystem->retractAlignMech(),
        liftSubsystem->positionCommand(120_deg, 0.0)->race(drivetrainSubsystem->hangPctCommand(0.0))->
        withTimeout(0.4_s),
        liftSubsystem->positionCommand(100_deg)->race(drivetrainSubsystem->hangDown(-1.0, -2.1_in)),
        liftSubsystem->positionCommand(25_deg)->race(drivetrainSubsystem->hangPctCommand(-0.54))->withTimeout(0.3_s),
        liftSubsystem->positionCommand(90_deg)->race(drivetrainSubsystem->hangPctCommand(1.0))->withTimeout(0.1_s),
        barToBarHang, barToBarHang
    });

    Trigger([]() { return pros::competition::is_disabled(); }).onTrue(drivetrainSubsystem->retractPto());

    Trigger([]() { return topIntakeSubsystem->ringPresentEject() && intakeOntoGoal->scheduled(); })
            .onFalse((new WaitCommand(30_ms))->andThen(new InstantCommand(
                []() mutable {
                    lastColor = topIntakeSubsystem->getRingColor();
                    primary.print(0, 0, "%d", lastColor);
                    if (static_cast<Alliance>(lastColor) != ALLIANCE && lastColor != RingColor::None)
                        ejectionPoints.
                                emplace_back(static_cast<int>(std::floor(topIntakeSubsystem->getPosition())) + 1);
                },
                {})));

    Trigger([]() mutable {
                return std::fmod(std::fmod(topIntakeSubsystem->getPosition(), 1.0) + 10.0, 1.0) > 0.38 &&
                       intakeOntoGoal->scheduled() &&
                       std::find(ejectionPoints.begin(), ejectionPoints.end(),
                                 static_cast<int>(std::floor(topIntakeSubsystem->getPosition()))) != ejectionPoints.
                       end();
            })
            .onTrue((new InstantCommand(
                    []() mutable {
                        std::erase(ejectionPoints, static_cast<int>(std::floor(topIntakeSubsystem->getPosition())));
                    },
                    {}))
                ->andThen(topIntakeSubsystem->pctCommand(-1.0)->withTimeout(0.07_s)->andThen(
                    new ScheduleCommand(intakeOntoGoal))));


    Trigger([]() mutable {
                return std::fmod(std::fmod(topIntakeSubsystem->getPosition(), 1.0) + 10.0, 1.0) > 0.3 && loadLB->
                       scheduled() &&
                       std::find(ejectionPoints.begin(), ejectionPoints.end(),
                                 static_cast<int>(std::floor(topIntakeSubsystem->getPosition()))) != ejectionPoints.
                       end();
            })
            .onTrue(
                (liftSubsystem->positionCommand(0_deg)->with(topIntakeSubsystem->pctCommand(1.0)))->withTimeout(0.1_s)->
                andThen(
                    (liftSubsystem->positionCommand(0_deg)->with(topIntakeSubsystem->pctCommand(-1.0)))->
                    withTimeout(0.07_s))->andThen(
                    new ScheduleCommand(loadLB)));

    primary.getTrigger(DIGITAL_X)->toggleOnTrue(drivetrainSubsystem->arcadeRecord(primary));

    primary.getTrigger(DIGITAL_L1)->whileTrue(new Command()); // LB up
    primary.getTrigger(DIGITAL_L2)->toggleOnTrue(new Command()); // LB down

    primary.getTrigger(DIGITAL_R2)->toggleOnTrue(intakeOntoGoal);
    primary.getTrigger(DIGITAL_R1)->onTrue(new Command()); // loading position

    primary.getTrigger(DIGITAL_RIGHT)->whileFalse(goalClampTrue);

    PathCommands::registerCommand("clamp", goalClampTrue);
    PathCommands::registerCommand("declamp", goalClampSubsystem->levelCommand(false));
}
