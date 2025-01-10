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
#include "goalClamp.h"
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

Command *loadOneRingHigh;
Command *loadOneRingLow;
Command *intakeOntoGoal;

Command *goalClampTrue;

Command *hang;
Command *barToBarHang;

CommandController primary(pros::controller_id_e_t::E_CONTROLLER_MASTER);
CommandController partner(pros::controller_id_e_t::E_CONTROLLER_PARTNER);

GpsSensor *gpsSensor;

bool outtakeWallStake = false;
bool hasRings = false;
std::vector<int> ejectionPoints{};
RingColor lastColor;

inline void subsystemInit() {
    TELEMETRY.setSerial(new pros::Serial(6, 921600));

    topIntake = new TopIntake({-12, 13}, pros::Distance(20));
    bottomIntake = new BottomIntake(pros::Motor(18));
    lift = new LiftSubsystem({-5, 7}, PID(4.5, 0.0, 3.0));
    goalClamp = new GoalClamp(pros::adi::DigitalOut('b'));
    drivetrain = new Drivetrain({-8, -16}, {3, 4}, {21}, {-2}, pros::Imu(14), pros::adi::DigitalOut('a'),
                                pros::adi::DigitalOut('c'), []() { return goalClamp->getLastValue(); });

    drivetrain->addLocalizationSensor(new Distance(CONFIG::DISTANCE_LEFT_OFFSET, 2438.0/2485.0, pros::Distance(15)));
    drivetrain->addLocalizationSensor(new Distance(CONFIG::DISTANCE_FRONT_OFFSET, 2438.0/2480.0, pros::Distance(1)));
    drivetrain->addLocalizationSensor(new Distance(CONFIG::DISTANCE_RIGHT_OFFSET, 2438.0/2505.0, pros::Distance(11)));
    drivetrain->addLocalizationSensor(new Distance(CONFIG::DISTANCE_BACK_OFFSET, 2438.0/2483.0, pros::Distance(9)));

    drivetrain->initUniform(-70_in, -70_in, 70_in, 70_in, 0_deg, false);

    CommandScheduler::registerSubsystem(drivetrain, drivetrain->tank(primary));
    CommandScheduler::registerSubsystem(
            topIntake, new ConditionalCommand(topIntake->pctCommand(0.0),
                                              TopIntakePositionCommand::fromReversePositionCommand(topIntake, 0.0, 0.0),
                                              [&]() { return hasRings; }));
    CommandScheduler::registerSubsystem(bottomIntake, bottomIntake->stopIntake());
    CommandScheduler::registerSubsystem(lift, new ConditionalCommand(lift->positionCommand(25.0_deg),
                                                                     lift->positionCommand(0.0_deg),
                                                                     [&]() { return hasRings; }));
    CommandScheduler::registerSubsystem(goalClamp, goalClamp->levelCommand(false));

    goalClampTrue = goalClamp->levelCommand(true);

    loadOneRingLow = new Sequence({
            new ParallelRaceGroup({bottomIntake->movePct(1.0), lift->positionCommand(0.0_deg),
                                   TopIntakePositionCommand::fromForwardPositionCommand(topIntake, 0.3, 0.0),
                                   new WaitUntilCommand([&]() { return topIntake->ringPresent(); })}),
            new ParallelRaceGroup({
                    bottomIntake->movePct(1.0),
                    lift->positionCommand(0.0_deg),
                    new ParallelCommandGroup({TopIntakePositionCommand::fromForwardPositionCommand(topIntake, 1.3),
                                              new WaitCommand(0.5_s)}),
            }),
    });
    loadOneRingHigh = new Sequence({
            new ParallelRaceGroup({bottomIntake->movePct(1.0), lift->positionCommand(CONFIG::WALL_STAKE_LOAD_HEIGHT),
                                   TopIntakePositionCommand::fromReversePositionCommand(topIntake, -0.50, 0.0),
                                   new WaitUntilCommand([&]() { return topIntake->ringPresent(); })}),
            // (new ParallelRaceGroup({bottomIntake->movePct(1.0),
            // lift->positionCommand(CONFIG::WALL_STAKE_LOAD_HEIGHT),
            //                         TopIntakePositionCommand::fromReversePositionCommand(topIntake, -0.45, 0.0)}))
            //         ->withTimeout(0.3_s),
            new ParallelRaceGroup({
                    bottomIntake->movePct(1.0),
                    lift->positionCommand(CONFIG::WALL_STAKE_LOAD_HEIGHT),
                    new ParallelCommandGroup({TopIntakePositionCommand::fromReversePositionCommand(topIntake, -1.50)}),
            }),
    });
    intakeOntoGoal = new ParallelCommandGroup({
            bottomIntake->movePct(1.0),
            lift->positionCommand(0.0_deg),
            topIntake->pctCommand(1.0),
            new InstantCommand([&]() { hasRings = false; }, {}),
    });

    barToBarHang =
            new Sequence({lift->positionCommand(35_deg)->race(drivetrain->hangUp(1.0, 7.2_in)),
                          lift->positionCommand(72_deg)->race(drivetrain->hangPctCommand(0.0))->withTimeout(0.25_s),
                          lift->positionCommand(72_deg)->race(drivetrain->hangDown(-1.0, 4_in)),
                          lift->positionCommand(117_deg)->race(drivetrain->hangDown(-1.0, -1.7_in)),
                          lift->positionCommand(30_deg)->race(drivetrain->hangDown(-1.0, -2.0_in)),
                          lift->positionCommand(25_deg)->race(drivetrain->hangPctCommand(-0.7))->withTimeout(0.3_s),
                          lift->positionCommand(90_deg)->race(drivetrain->hangPctCommand(1.0))->withTimeout(0.1_s)});
    hang = new Sequence({drivetrain->activatePto(), drivetrain->retractAlignMech(),
                         lift->moveToPosition(110_deg)->race(drivetrain->hangPctCommand(0.0))->withTimeout(0.4_s),
                         lift->positionCommand(110_deg)->race(drivetrain->hangDown(-1.0, -1.7_in)),
                         lift->positionCommand(30_deg)->race(drivetrain->hangDown(-1.0, -2.0_in)),
                         lift->positionCommand(25_deg)->race(drivetrain->hangPctCommand(-0.6))->withTimeout(0.3_s),
                         lift->positionCommand(90_deg)->race(drivetrain->hangPctCommand(1.0))->withTimeout(0.1_s),
                         barToBarHang, barToBarHang});

    Trigger([]() { return pros::competition::is_disabled(); }).onTrue(drivetrain->retractPto());

    Trigger([]() { return topIntake->ringPresent() && intakeOntoGoal->scheduled(); })
            .onFalse(new InstantCommand(
                    []() mutable {
                        lastColor = topIntake->getRingColor();
                        primary.print(0, 0, "%d", lastColor);
                        if (lastColor != ALLIANCE && lastColor != RingColor::None)
                            ejectionPoints.emplace_back(static_cast<int>(std::floor(topIntake->getPosition())) + 1);
                    },
                    {}));

    Trigger([]() mutable {
        return std::fmod(std::fmod(topIntake->getPosition(), 1.0) + 10.0, 1.0) > 0.4 && intakeOntoGoal->scheduled() &&
               std::find(ejectionPoints.begin(), ejectionPoints.end(),
                         static_cast<int>(std::floor(topIntake->getPosition()))) != ejectionPoints.end();
    })
            .onTrue((new InstantCommand(
                             []() mutable {
                                 std::erase(ejectionPoints, static_cast<int>(std::floor(topIntake->getPosition())));
                             },
                             {}))
                            ->andThen(topIntake->pctCommand(-1.0)->withTimeout(0.07_s)->andThen(
                                    new ScheduleCommand(intakeOntoGoal))));

    primary.getTrigger(DIGITAL_X)->toggleOnTrue(drivetrain->arcadeRecord(primary));

    primary.getTrigger(DIGITAL_L1)
            ->toggleOnTrue(new Sequence(
                    {new InstantCommand([&]() { hasRings = true; }, {}), loadOneRingHigh, loadOneRingHigh}));

    primary.getTrigger(DIGITAL_L2)
            ->toggleOnTrue(new Sequence(
                    {new InstantCommand([&]() { hasRings = false; }, {}), loadOneRingLow, loadOneRingLow}));

    primary.getTrigger(DIGITAL_R2)->toggleOnTrue(intakeOntoGoal);
    primary.getTrigger(DIGITAL_R1)
            ->onTrue(new Sequence({new InstantCommand([&]() { outtakeWallStake = false; }, {}),
                                   new ParallelRaceGroup({
                                           bottomIntake->movePct(0.0),
                                           lift->moveToPosition(CONFIG::WALL_STAKE_SCORE_HEIGHT),
                                           TopIntakePositionCommand::fromReversePositionCommand(topIntake, -0.3, 0.0),
                                   }),
                                   new ParallelRaceGroup({bottomIntake->movePct(0.0),
                                                          lift->positionCommand(CONFIG::WALL_STAKE_SCORE_HEIGHT),
                                                          topIntake->pctCommand(0.0), new WaitUntilCommand([&]() {
                                                              return primary.get_digital(DIGITAL_Y);
                                                          })}),
                                   new InstantCommand(
                                           [&]() {
                                               outtakeWallStake = true;
                                               hasRings = false;
                                           },
                                           {}),
                                   new ParallelCommandGroup({
                                           bottomIntake->movePct(0.0),
                                           lift->positionCommand(CONFIG::WALL_STAKE_SCORE_HEIGHT),
                                           topIntake->pctCommand(-1.0),
                                   })}))
            ->onFalse(new ParallelRaceGroup({
                    bottomIntake->movePct(0.0),
                    lift->moveToPosition(CONFIG::WALL_STAKE_LOAD_HEIGHT, 10_deg),
                    new ConditionalCommand(topIntake->pctCommand(1.0), topIntake->pctCommand(0.0),
                                           [&]() { return outtakeWallStake; }),
            }));

    primary.getTrigger(DIGITAL_RIGHT)->toggleOnTrue(goalClampTrue);

    primary.getTrigger(DIGITAL_DOWN)->whileTrue(hang);

    primary.getTrigger(DIGITAL_B)->onTrue(drivetrain->releaseString()->with(lift->positionCommand(65_deg)));


    partner.getTrigger(DIGITAL_DOWN)->whileTrue(hang);
    partner.getTrigger(DIGITAL_Y)->onTrue(drivetrain->releaseString()->with(lift->positionCommand(65_deg)));

    partner.getTrigger(DIGITAL_A)->whileTrue(new ParallelCommandGroup(
            {new InstantCommand([&]() { hasRings = false; }, {}), bottomIntake->movePct(0.8),
             lift->positionCommand(CONFIG::WALL_STAKE_LOAD_HEIGHT), topIntake->pctCommand(-1.0)}));

    partner.getTrigger(DIGITAL_RIGHT)->onTrue(new InstantCommand([]() { ALLIANCE = RED; }, {}));
    partner.getTrigger(DIGITAL_LEFT)->onTrue(new InstantCommand([]() { ALLIANCE = BLUE; }, {}));

    Trigger([&]() { return abs(partner.get_analog(ANALOG_RIGHT_Y)) > 15; }, CommandScheduler::getTeleopEventLoop())
            .whileTrue(topIntake->controllerCommand(&partner));
    partner.getTrigger(DIGITAL_X)->whileTrue(bottomIntake->movePct(1.0));
    partner.getTrigger(DIGITAL_B)->whileTrue(bottomIntake->movePct(-1.0));

    partner.getTrigger(DIGITAL_L1)
            ->toggleOnTrue(new Sequence({new InstantCommand([&]() { hasRings = true; }, {}),
                                         new Sequence({loadOneRingHigh, loadOneRingHigh})}));

    partner.getTrigger(DIGITAL_L2)
            ->toggleOnTrue(new Sequence(
                    {new InstantCommand([&]() { hasRings = false; }, {}), loadOneRingLow, loadOneRingLow}));

    partner.getTrigger(DIGITAL_R2)->toggleOnTrue(intakeOntoGoal);
    partner.getTrigger(DIGITAL_R1)
            ->onTrue(new Sequence({new InstantCommand([&]() { outtakeWallStake = false; }, {}),
                                   new ParallelRaceGroup({
                                           bottomIntake->movePct(0.0),
                                           lift->moveToPosition(CONFIG::WALL_STAKE_SCORE_HEIGHT),
                                           topIntake->pctCommand(0.0),
                                   }),
                                   new ParallelRaceGroup({bottomIntake->movePct(0.0),
                                                          lift->positionCommand(CONFIG::WALL_STAKE_SCORE_HEIGHT),
                                                          topIntake->pctCommand(0.0), new WaitUntilCommand([&]() {
                                                              return primary.get_digital(DIGITAL_Y);
                                                          })}),
                                   new InstantCommand(
                                           [&]() {
                                               outtakeWallStake = true;
                                               hasRings = false;
                                           },
                                           {}),
                                   new ParallelCommandGroup({
                                           bottomIntake->movePct(0.0),
                                           lift->positionCommand(CONFIG::WALL_STAKE_SCORE_HEIGHT),
                                           topIntake->pctCommand(-1.0),
                                   })}))
            ->onFalse(new ParallelRaceGroup({
                    bottomIntake->movePct(0.0),
                    lift->moveToPosition(0_deg),
                    new ConditionalCommand(topIntake->pctCommand(1.0), topIntake->pctCommand(0.0),
                                           [&]() { return outtakeWallStake; }),
            }));

    PathCommands::registerCommand("intakeNeutralStakes", loadOneRingHigh->repeatedly());
    PathCommands::registerCommand(
            "intakeNeutralStakesElim",
            loadOneRingHigh->andThen(bottomIntake->movePct(1.0)->with(
                    topIntake->pctCommand(0.0)->with(lift->positionCommand(CONFIG::WALL_STAKE_SCORE_HEIGHT)))));
    PathCommands::registerCommand("intakeOneNeutralStakes",
                                  loadOneRingHigh->andThen(lift->moveToPosition(30_deg)->with(
                                          bottomIntake->movePct(0.0)->with(topIntake->pctCommand(0.0)))));

    PathCommands::registerCommand("bottomIntakeOffTopOn", bottomIntake->movePct(-0.07)->with(topIntake->pctCommand(1.0)));

    PathCommands::registerCommand("bottomOuttake",
                                  bottomIntake->movePct(-1.0)->with(lift->positionCommand(0.0))->withTimeout(0.5_s));
    PathCommands::registerCommand("liftArm",
                                  new ParallelRaceGroup({
                                          bottomIntake->movePct(0.0),
                                          lift->positionCommand(CONFIG::WALL_STAKE_SCORE_HEIGHT),
                                          topIntake->pctCommand(0.0),
                                  }));
    PathCommands::registerCommand("liftArm2", new ParallelCommandGroup({
                                                      bottomIntake->movePct(0.0),
                                                      lift->positionCommand(CONFIG::WALL_STAKE_SCORE_HEIGHT),
                                                      topIntake->pctCommand(0.0),
                                              }));
    PathCommands::registerCommand(
            "intakeAllianceStakes",
            new Sequence(
                    {loadOneRingLow,
                     new ParallelCommandGroup({TopIntakePositionCommand::fromForwardPositionCommand(topIntake, 1.2),
                                               new WaitCommand(0.5_s), bottomIntake->movePct(0.0)}),
                     new ParallelCommandGroup({topIntake->pctCommand(0.0), bottomIntake->movePct(-0.2)})}));
    PathCommands::registerCommand("intakeGoal", intakeOntoGoal);
    PathCommands::registerCommand("intakeGoalNoEject", new ParallelCommandGroup({
                                                               bottomIntake->movePct(1.0),
                                                               lift->positionCommand(0.0_deg),
                                                               topIntake->pctCommand(1.0),
                                                               new InstantCommand([&]() { hasRings = false; }, {}),
                                                       }));
    PathCommands::registerCommand("intakeGoalSlow", new ParallelCommandGroup({
                                                            bottomIntake->movePct(0.7),
                                                            lift->positionCommand(0.0_deg),
                                                            topIntake->pctCommand(1.0),
                                                            new InstantCommand([&]() { hasRings = false; }, {}),
                                                    }));
    PathCommands::registerCommand(
            "stopIntake",
            new ParallelCommandGroup({TopIntakePositionCommand::fromForwardPositionCommand(topIntake, 0.9, 0.0),
                                      bottomIntake->movePct(1.0), lift->positionCommand(0.0)}));
    PathCommands::registerCommand(
            "stopIntake3",
            new ParallelCommandGroup({TopIntakePositionCommand::fromForwardPositionCommand(topIntake, 3.8, 0.0),
                                      bottomIntake->movePct(1.0), lift->positionCommand(0.0)}));
    PathCommands::registerCommand("clamp", goalClamp->levelCommand(true));
    PathCommands::registerCommand("declamp", goalClamp->levelCommand(false));
    PathCommands::registerCommand("declampElim", goalClamp->levelCommand(false));
    PathCommands::registerCommand(
            "indexTwo", new ParallelCommandGroup({TopIntakePositionCommand::fromForwardPositionCommand(topIntake, 1.7),
                                                  bottomIntake->movePct(1.0), lift->positionCommand(0.0)}));
    PathCommands::registerCommand(
            "dejam",
            new ParallelCommandGroup({bottomIntake->movePct(0.8), lift->positionCommand(CONFIG::WALL_STAKE_LOAD_HEIGHT),
                                      topIntake->pctCommand(-1.0)}));
    PathCommands::registerCommand("dejam2",
                                  new ParallelCommandGroup({bottomIntake->movePct(-1.0),
                                                            lift->positionCommand(CONFIG::WALL_STAKE_LOAD_HEIGHT),
                                                            topIntake->pctCommand(0.0)}));
    PathCommands::registerCommand("awpLiftUp", lift->positionCommand(30_deg)->with(topIntake->pctCommand(-0.3)));
    PathCommands::registerCommand("intake2andIndex", loadOneRingLow->andThen(loadOneRingLow));
    PathCommands::registerCommand("outtakeBottom", bottomIntake->movePct(-0.7)->with(lift->positionCommand(35_deg)));
    PathCommands::registerCommand("hangRelease", drivetrain->releaseString()->with(lift->positionCommand(65_deg)));
}
