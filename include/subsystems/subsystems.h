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
inline Command *gripBar;
inline Command *letOutString;

inline CommandController primary(pros::controller_id_e_t::E_CONTROLLER_MASTER);
inline CommandController partner(pros::controller_id_e_t::E_CONTROLLER_PARTNER);

inline std::vector<int> ejectionPoints{};
bool loadingLB = false;

inline void subsystemInit() {
    TELEMETRY.setSerial(new pros::Serial(0, 921600));

    topIntakeSubsystem = new TopIntakeSubsystem({3}, pros::AIVision(21));
    bottomIntakeSubsystem = new MotorSubsystem(pros::Motor(-2));
    liftSubsystem = new LiftSubsystem({-1}, PID(2.0, 0.0, 0.0));
    goalClampSubsystem = new SolenoidSubsystem(pros::adi::DigitalOut('c'));
    hangSubsystem = new SolenoidSubsystem({pros::adi::DigitalOut('d'), pros::adi::DigitalOut('b')});
    // 'd' left, 'b' right
    drivetrainSubsystem = new DrivetrainSubsystem({-11, 12, -13}, {20, -19, 18}, {}, {}, pros::Imu(9),
                                                  pros::adi::DigitalOut('a'), pros::Rotation(8), []() {
                                                      return goalClampSubsystem->getLastValue();
                                                  }); // wheels listed back to front; 8 for rotation sensor on pto

    drivetrainSubsystem->addLocalizationSensor(new Distance(CONFIG::DISTANCE_LEFT_OFFSET, 2388.0 / 2445.0,
                                                            pros::Distance(5)));
    drivetrainSubsystem->addLocalizationSensor(new Distance(CONFIG::DISTANCE_FRONT_OFFSET, 2388.0 / 2428.0,
                                                            pros::Distance(6)));
    drivetrainSubsystem->addLocalizationSensor(new Distance(CONFIG::DISTANCE_RIGHT_OFFSET, 2388.0 / 2415.0,
                                                            pros::Distance(7)));
    drivetrainSubsystem->addLocalizationSensor(new Distance(CONFIG::DISTANCE_BACK_OFFSET, 2388.0 / 2443.0,
                                                            pros::Distance(10)));

    drivetrainSubsystem->initUniform(-70_in, -70_in, 70_in, 70_in, 0_deg, false);

    CommandScheduler::registerSubsystem(drivetrainSubsystem, drivetrainSubsystem->tank(primary));
    CommandScheduler::registerSubsystem(topIntakeSubsystem,
                                        TopIntakePositionCommand::fromClosePositionCommand(
                                            topIntakeSubsystem, 0.0, 0.0));
    CommandScheduler::registerSubsystem(bottomIntakeSubsystem, bottomIntakeSubsystem->stopIntake());
    CommandScheduler::registerSubsystem(liftSubsystem, liftSubsystem->positionCommand(5_deg, 0.0));
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

    letOutString = new ParallelRaceGroup({
        drivetrainSubsystem->hangOut(1.0, 8.0_in),
        hangSubsystem->levelCommand(false)->with(liftSubsystem->positionCommand(50_deg))->until([]() {
            return drivetrainSubsystem->getStringDistance() > 5_in;
        })->andThen(hangSubsystem->levelCommand(true)->with(liftSubsystem->positionCommand(70_deg))),
    });

    gripBar = new Sequence({
        new ParallelRaceGroup({
            drivetrainSubsystem->hangIn(1.0, 0.0_in),
            hangSubsystem->levelCommand(true),
            liftSubsystem->positionCommand(70_deg)
        }),
        new ParallelRaceGroup({
            drivetrainSubsystem->hangPctCommand(0.2),
            hangSubsystem->levelCommand(false),
            liftSubsystem->positionCommand(70_deg)
        })
    });

    barToBarHang =
            new Sequence({
                letOutString,
                gripBar
            });
    hang = new Sequence({
        drivetrainSubsystem->activatePto(),
        gripBar
    });

    Trigger([]() {
                return topIntakeSubsystem->getRing() != RingColor::None && (
                           intakeOntoGoal->scheduled() || loadLB->scheduled());
            })
            .onTrue((new InstantCommand(
                []() mutable {
                    auto ring = topIntakeSubsystem->getRing();
                    std::cout << "RING: " << ring << std::endl;
                    if (static_cast<Alliance>(ring) != ALLIANCE && ring != RingColor::None)
                        ejectionPoints.
                                emplace_back(static_cast<int>(std::floor(topIntakeSubsystem->getPosition())) + 1);
                },
                {}))->andThen(new ConditionalCommand(liftSubsystem->positionCommand(0.0, 0.0), new InstantCommand([]() {
                                                     }, {}), []() { return !loadLB->scheduled(); })));

    Trigger([]() {
                return std::fmod(std::fmod(topIntakeSubsystem->getPosition(), 1.0) + 10.0, 1.0) > 0.38 &&
                       (intakeOntoGoal->scheduled() || loadLB->scheduled()) &&
                       std::find(ejectionPoints.begin(), ejectionPoints.end(),
                                 static_cast<int>(std::floor(topIntakeSubsystem->getPosition()))) != ejectionPoints.
                       end();
            })
            .onTrue((new InstantCommand(
                    []() mutable {
                        loadingLB = loadLB->scheduled();
                        std::cout << "RING: eject " << std::endl;
                        std::erase(ejectionPoints, static_cast<int>(std::floor(topIntakeSubsystem->getPosition())));
                    },
                    {}))
                ->andThen(topIntakeSubsystem->pctCommand(-1.0)->withTimeout(0.07_s)->andThen(
                    new ConditionalCommand(new ScheduleCommand(loadLB), new ScheduleCommand(intakeOntoGoal),
                                           [] { return loadingLB; }))));

    primary.getTrigger(DIGITAL_X)->toggleOnTrue(drivetrainSubsystem->arcadeRecord(primary));
    primary.getTrigger(DIGITAL_A)->whileTrue(hang);

    primary.getTrigger(DIGITAL_L1)->whileTrue(liftSubsystem->pctCommand(1.0))->onFalse(
        liftSubsystem->holdPositionCommand()); // LB up
    primary.getTrigger(DIGITAL_L2)->whileTrue(liftSubsystem->pctCommand(-1.0))->onFalse(
        new ConditionalCommand(liftSubsystem->holdPositionCommand(), liftSubsystem->positionCommand(0_deg),
                               []() { return liftSubsystem->getPosition() > 90_deg; })); // LB down

    primary.getTrigger(DIGITAL_R2)->toggleOnTrue(intakeOntoGoal);
    primary.getTrigger(DIGITAL_R1)->toggleOnTrue(loadLB); // loading position

    primary.getTrigger(DIGITAL_DOWN)->toggleOnTrue(hangSubsystem->levelCommand(true));
    primary.getTrigger(DIGITAL_UP)->onTrue(drivetrainSubsystem->activatePto());
    primary.getTrigger(DIGITAL_LEFT)->onTrue(drivetrainSubsystem->retractPto());
    primary.getTrigger(DIGITAL_RIGHT)->whileFalse(goalClampTrue);

    PathCommands::registerCommand("clamp", goalClampTrue);
    PathCommands::registerCommand("declamp", goalClampSubsystem->levelCommand(false));
}
