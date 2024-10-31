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
#include "commands/ramsete.h"
#include "commands/rotate.h"
#include "drivetrain.h"
#include "goalClamp.h"
#include "hang.h"
#include "hook.h"
#include "lift.h"
#include "localization/distance.h"
#include "localization/gps.h"
#include "localization/line.h"
#include "motionProfiling/pathCommands.h"
#include "pros/adi.hpp"
#include "topIntake.h"

#include <queue>

Drivetrain *drivetrain;
TopIntake *topIntake;
BottomIntake *bottomIntake;
LiftSubsystem *lift;
GoalClamp *goalClamp;
Hook *hook;
Hang *hang;

Command *loadOneRingHigh;
Command *loadOneRingLow;
Command *intakeOntoGoal;

Command *goalClampTrue;

CommandController primary(pros::controller_id_e_t::E_CONTROLLER_MASTER);
CommandController partner(pros::controller_id_e_t::E_CONTROLLER_PARTNER);

pros::Distance goalClampDistanceSensor(20);

GpsSensor *gpsSensor;

bool outtakeWallStake = false;
bool hasRings = false;
std::vector<int> ejectionPoints{};
RingColor lastColor;

inline void subsystemInit() {
    TELEMETRY.setSerial(new pros::Serial(19, 921600));

    drivetrain = new Drivetrain({-2, -3}, {6, 7}, {4}, {-9}, pros::Imu(16));
    topIntake = new TopIntake(pros::Motor(-5), pros::Distance(21));
    bottomIntake = new BottomIntake(pros::Motor(-10));
    lift = new LiftSubsystem(pros::Motor(1), PID(30.0, 0.0, 50.0));
    goalClamp = new GoalClamp(pros::adi::DigitalOut('a'));
    hook = new Hook(pros::Motor(-8));
    hang = new Hang(pros::adi::DigitalOut('c'));

    drivetrain->addLocalizationSensor(new Distance(CONFIG::DISTANCE_LEFT_OFFSET, pros::Distance(15)));
    drivetrain->addLocalizationSensor(new Distance(CONFIG::DISTANCE_BACK_OFFSET, pros::Distance(14)));
    drivetrain->addLocalizationSensor(new Distance(CONFIG::DISTANCE_RIGHT_OFFSET, pros::Distance(11)));

    drivetrain->initUniform(-70_in, -70_in, 70_in, 70_in, 0_deg, false);

    CommandScheduler::registerSubsystem(drivetrain, drivetrain->arcade(primary));
    CommandScheduler::registerSubsystem(topIntake, new ConditionalCommand(topIntake->stopIntake(),
                                                                          topIntake->positionCommandFwd(0.0),
                                                                          [&]() { return hasRings; }));
    CommandScheduler::registerSubsystem(bottomIntake, bottomIntake->stopIntake());
    CommandScheduler::registerSubsystem(lift, new ConditionalCommand(lift->positionCommand(3.0_deg),
                                                                     lift->positionCommand(0.0_deg),
                                                                     [&]() { return hasRings; }));
    CommandScheduler::registerSubsystem(goalClamp, goalClamp->levelCommand(false));
    CommandScheduler::registerSubsystem(hook, hook->positionCommand(0.0));
    CommandScheduler::registerSubsystem(hang, hang->levelCommand(false));

    goalClampTrue = goalClamp->levelCommand(true);

    loadOneRingLow = new Sequence({
            new ParallelRaceGroup({bottomIntake->movePct(1.0), lift->positionCommand(0.0_deg),
                                   topIntake->positionCommandFwd(0.3),
                                   new WaitUntilCommand([&]() { return topIntake->ringPresent(); })}),
            new ParallelRaceGroup({
                    bottomIntake->movePct(1.0),
                    lift->positionCommand(0.0_deg),
                    new ParallelCommandGroup({topIntake->moveToPositionFwd(1.3), new WaitCommand(0.5_s)}),
            }),
    });
    loadOneRingHigh = new Sequence({
            new ParallelRaceGroup({bottomIntake->movePct(1.0), lift->positionCommand(5.0_deg),
                                   topIntake->positionCommandRwd(-0.1),
                                   new WaitUntilCommand([&]() { return topIntake->ringPresent(); })}),
            new ParallelRaceGroup({
                    bottomIntake->movePct(1.0),
                    lift->positionCommand(5.0_deg),
                    new ParallelCommandGroup({topIntake->moveToPositionRwd(-1.1), new WaitCommand(0.5_s)}),
            }),
    });
    intakeOntoGoal = new ParallelCommandGroup({
            bottomIntake->movePct(1.0),
            lift->positionCommand(0.0_deg),
            topIntake->movePct(1.0),
            new InstantCommand([&]() { hasRings = false; }, {}),
    });

    primary.getTrigger(DIGITAL_R1)->whileTrue(drivetrain->arcade(partner, 0.4, 0.5));
    primary.getTrigger(DIGITAL_R2)->whileTrue(drivetrain->arcade(partner, 0.6, 0.5));

    PathCommands::registerCommand("intakeNeutralStakes", loadOneRingHigh->andThen(loadOneRingHigh));
    PathCommands::registerCommand("liftArm", new ParallelRaceGroup({
                                                     bottomIntake->movePct(0.0),
                                                     lift->positionCommand(33_deg),
                                                     topIntake->positionCommandClose(-0.1),
                                             }));
    PathCommands::registerCommand("liftArm2", new ParallelRaceGroup({
                                                      bottomIntake->movePct(0.0),
                                                      lift->positionCommand(33_deg),
                                                      topIntake->movePct(0.0),
                                              }));
    PathCommands::registerCommand(
            "intakeAllianceStakes",
            new Sequence({loadOneRingLow,
                          new ParallelCommandGroup({topIntake->moveToPositionFwd(1.3), new WaitCommand(0.5_s),
                                                    bottomIntake->movePct(0.0)}),
                          new ParallelCommandGroup({topIntake->movePct(0.0), bottomIntake->movePct(-0.1)})}));
    PathCommands::registerCommand("scoreNeutral", new Sequence({new ParallelRaceGroup({
                                                                        bottomIntake->movePct(0.0),
                                                                        lift->moveToPosition(33_deg, 0.3_deg),
                                                                        topIntake->movePct(0.0),
                                                                }),
                                                                new ParallelRaceGroup({
                                                                        bottomIntake->movePct(0.0),
                                                                        lift->moveToPosition(33_deg, 1_deg),
                                                                        topIntake->movePct(0.0),
                                                                }),
                                                                new ParallelCommandGroup({
                                                                        bottomIntake->movePct(0.0),
                                                                        lift->positionCommand(33_deg),
                                                                        topIntake->movePct(-1.0),
                                                                })}));
    PathCommands::registerCommand("intakeGoal", intakeOntoGoal);
    PathCommands::registerCommand("stopIntake",
                                  new ParallelCommandGroup({topIntake->moveToPositionFwd(1.1),
                                                            bottomIntake->movePct(1.0), lift->positionCommand(0.0)}));
    PathCommands::registerCommand("clamp", goalClamp->levelCommand(true));
    PathCommands::registerCommand("declamp", goalClamp->levelCommand(false));
    PathCommands::registerCommand(
            "hang",
            new ParallelCommandGroup({hang->levelCommand(true), new ScheduleCommand(topIntake->moveToPositionFwd(0.0)),
                                      lift->positionCommand(0.0_deg)}));
    PathCommands::registerCommand("indexTwo",
                                  new ParallelCommandGroup({topIntake->moveToPositionFwd(1.7),
                                                            bottomIntake->movePct(1.0), lift->positionCommand(0.0)}));
    PathCommands::registerCommand("dejam",
                                  new ParallelCommandGroup({bottomIntake->movePct(0.8), lift->positionCommand(7.0_deg),
                                                            topIntake->movePct(1.0)}));
    PathCommands::registerCommand("dejam2",
                                  new ParallelCommandGroup({bottomIntake->movePct(-1.0), lift->positionCommand(7.0_deg),
                                                            topIntake->movePct(0.0)}));
    PathCommands::registerCommand("awpLiftUp", lift->positionCommand(30_deg)->with(topIntake->movePct(-0.3)));
}
