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

inline std::vector<int> ejectionPoints{};
inline RingColor lastColor;

inline void subsystemInit() {
    TELEMETRY.setSerial(new pros::Serial(0, 921600));

    topIntakeSubsystem = new TopIntakeSubsystem({3}, pros::Distance(20), pros::AIVision(14));
    bottomIntakeSubsystem = new MotorSubsystem(pros::Motor(-2));
    liftSubsystem = new LiftSubsystem({-1}, PID(4.5, 0.0, 3.0));
    goalClampSubsystem = new SolenoidSubsystem(pros::adi::DigitalOut('c'));
    hangSubsystem = new SolenoidSubsystem({pros::adi::DigitalOut('d'), pros::adi::DigitalOut('b')}); // 'd' left, 'b' right
    drivetrainSubsystem = new DrivetrainSubsystem({-20, 19, -18}, {-10, 12, -13}, {}, {}, pros::Imu(14),
                                                  pros::adi::DigitalOut('a'), []() {
                                                      return goalClampSubsystem->getLastValue();
                                                  }); // wheels listed back to front

    drivetrainSubsystem->addLocalizationSensor(new Distance(CONFIG::DISTANCE_LEFT_OFFSET, 2438.0 / 2485.0,
                                                            pros::Distance(0)));
    drivetrainSubsystem->addLocalizationSensor(new Distance(CONFIG::DISTANCE_FRONT_OFFSET, 2438.0 / 2480.0,
                                                            pros::Distance(6)));
    drivetrainSubsystem->addLocalizationSensor(new Distance(CONFIG::DISTANCE_RIGHT_OFFSET, 2438.0 / 2505.0,
                                                            pros::Distance(0)));
    drivetrainSubsystem->addLocalizationSensor(new Distance(CONFIG::DISTANCE_BACK_OFFSET, 2438.0 / 2483.0,
                                                            pros::Distance(0)));

    drivetrainSubsystem->initUniform(-70_in, -70_in, 70_in, 70_in, 0_deg, false);

    CommandScheduler::registerSubsystem(drivetrainSubsystem, drivetrainSubsystem->tank(primary));
    CommandScheduler::registerSubsystem(topIntakeSubsystem, topIntakeSubsystem->pctCommand(0.0));
    CommandScheduler::registerSubsystem(bottomIntakeSubsystem, bottomIntakeSubsystem->stopIntake());
    CommandScheduler::registerSubsystem(liftSubsystem, liftSubsystem->positionCommand(0.0_deg));
    CommandScheduler::registerSubsystem(goalClampSubsystem, goalClampSubsystem->levelCommand(false));
    CommandScheduler::registerSubsystem(hangSubsystem, hangSubsystem->levelCommand(false));

    goalClampTrue = goalClampSubsystem->levelCommand(true);

    intakeOntoGoal = new ParallelCommandGroup({
        bottomIntakeSubsystem->pctCommand(1.0), topIntakeSubsystem->pctCommand(1.0)
    });
    loadLB = new Sequence({
        new ParallelRaceGroup({
            bottomIntakeSubsystem->pctCommand(1.0),
            topIntakeSubsystem->pctCommand(1.0),
            liftSubsystem->positionCommand(CONFIG::WALL_STAKE_LOAD_HEIGHT, 0.0),
            new WaitCommand(500_ms)
        }),
        new ParallelRaceGroup({
            bottomIntakeSubsystem->pctCommand(1.0),
            topIntakeSubsystem->pctCommand(1.0)->until([]() { return topIntakeSubsystem->stalled(200_ms); }),
            liftSubsystem->positionCommand(CONFIG::WALL_STAKE_LOAD_HEIGHT, 0.0),
        }),
        new ParallelRaceGroup({
            bottomIntakeSubsystem->pctCommand(0.0), topIntakeSubsystem->pctCommand(-1.0),
            liftSubsystem->positionCommand(CONFIG::WALL_STAKE_LOAD_HEIGHT, 0.0), new WaitCommand(50_ms)
        }),
        new ParallelRaceGroup({
            bottomIntakeSubsystem->pctCommand(0.0), topIntakeSubsystem->pctCommand(1.0),
            liftSubsystem->positionCommand(CONFIG::WALL_STAKE_LOAD_HEIGHT, 0.0), new WaitCommand(100_ms)
        }),
        new ScheduleCommand(liftSubsystem->positionCommand(CONFIG::WALL_STAKE_PRIME_HEIGHT))
    });

    barToBarHang =
            new Sequence({
                hangSubsystem->levelCommand(false)->race(drivetrainSubsystem->hangUp(1.0, 8.32_in)),
                hangSubsystem->levelCommand(true)->race(drivetrainSubsystem->hangPctCommand(0.0))->withTimeout(
                    0.35_s),
                hangSubsystem->levelCommand(true)->race(drivetrainSubsystem->hangDown(-1.0, 4_in)),
                hangSubsystem->levelCommand(true)->race(drivetrainSubsystem->hangDown(-1.0, -2.5_in)),
                hangSubsystem->levelCommand(false)->race(drivetrainSubsystem->hangPctCommand(-0.57))->
                withTimeout(0.3_s),
                hangSubsystem->levelCommand(false)->race(drivetrainSubsystem->hangPctCommand(1.0))->withTimeout(
                    0.1_s)
            });
    hang = new Sequence({
        drivetrainSubsystem->activatePto(),
        hangSubsystem->levelCommand(true)->race(drivetrainSubsystem->hangPctCommand(0.0))->
        withTimeout(0.4_s),
        hangSubsystem->levelCommand(true)->race(drivetrainSubsystem->hangDown(-1.0, -2.1_in)),
        hangSubsystem->levelCommand(false)->race(drivetrainSubsystem->hangPctCommand(-0.54))->withTimeout(0.3_s),
        hangSubsystem->levelCommand(true)->race(drivetrainSubsystem->hangPctCommand(1.0))->withTimeout(0.1_s),
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

    primary.getTrigger(DIGITAL_L1)->whileTrue(liftSubsystem->pctCommand(1.0))->onFalse(
        liftSubsystem->holdPositionCommand()); // LB up
    primary.getTrigger(DIGITAL_L2)->whileTrue(liftSubsystem->pctCommand(-1.0))->onFalse(
        new ConditionalCommand(liftSubsystem->holdPositionCommand(), liftSubsystem->positionCommand(0_deg),
                               []() { return liftSubsystem->getPosition() > 10_deg; })); // LB down

    primary.getTrigger(DIGITAL_R2)->toggleOnTrue(intakeOntoGoal);
    primary.getTrigger(DIGITAL_R1)->toggleOnTrue(loadLB); // loading position

    primary.getTrigger(DIGITAL_RIGHT)->whileFalse(goalClampTrue);

    PathCommands::registerCommand("clamp", goalClampTrue);
    PathCommands::registerCommand("declamp", goalClampSubsystem->levelCommand(false));
}
