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
#include "commands/lift/trapPosition.h"
#include "commands/ramsete.h"
#include "commands/rotate.h"
#include "drivetrain.h"
#include "lift.h"
#include "localization/distance.h"
#include "motionProfiling/pathCommands.h"
#include "pros/adi.hpp"
#include "solenoidSubsystem.h"
#include "topIntake.h"

#include <algorithm>
#include <queue>

inline DrivetrainSubsystem* drivetrainSubsystem;
inline TopIntakeSubsystem* topIntakeSubsystem;
inline MotorSubsystem* bottomIntakeSubsystem;
inline LiftSubsystem* liftSubsystem;
inline SolenoidSubsystem* goalClampSubsystem;
inline SolenoidSubsystem* hangSubsystem;

inline Command* loadLB;
inline Command* goalClampTrue;
inline Command* hang;
inline Command* barToBarHang;
inline Command* gripBar;
inline Command* letOutString;
inline Command* basicLoadLB;
inline Command* intakeNoEject;
inline Command* topIntakeWithEject;
inline Command* intakeWithEject;
inline Command* bottomOuttakeWithEject;
inline Command* hangRelease;
inline Command* hangIdle;
inline Command* cornerClearIntakeSequence;

inline CommandController primary(pros::controller_id_e_t::E_CONTROLLER_MASTER);
inline CommandController partner(pros::controller_id_e_t::E_CONTROLLER_PARTNER);

inline Trigger* negatedHang;
inline Trigger* negatedLBLoad;
inline Trigger* liftLow;

bool hangReleased = false;

inline void initializeController()
{
    primary.getTrigger(DIGITAL_X)->toggleOnTrue(drivetrainSubsystem->arcade(primary));
    primary.getTrigger(DIGITAL_A)->whileTrue(bottomIntakeSubsystem->pctCommand(-1.0));

    primary.getTrigger(DIGITAL_R1)
           ->andOther(primary.getTrigger(DIGITAL_L1))
           ->andOther(primary.getTrigger(DIGITAL_R2))
           ->andOther(primary.getTrigger(DIGITAL_L2))
           ->onTrue(
               (new InstantCommand([]() { hangReleased = true; }, {}))->andThen(new ScheduleCommand(hangRelease)));

    negatedHang = ((new Trigger(hangRelease))->orOther(new Trigger(hangIdle)))->negate();
    negatedLBLoad = (new Trigger(loadLB))->negate();

    liftLow = new Trigger([] () { return liftSubsystem->getPosition() < 50_deg; });

    negatedHang->negate()->onTrue((new InstantCommand([]() { hangReleased = true; }, {})));

    primary.getTrigger(DIGITAL_R1)->andOther(primary.getTrigger(DIGITAL_R2)->negate())->
            andOther(primary.getTrigger(DIGITAL_L1)->negate())->andOther(primary.getTrigger(DIGITAL_L2)->negate())->
            andOther(negatedHang->negate())->onTrue(hangIdle)->onFalse(hangRelease);


    primary.getTrigger(DIGITAL_L1)
           ->andOther(primary.getTrigger(DIGITAL_L2)->negate())
           ->andOther(primary.getTrigger(DIGITAL_R1)->negate())
           ->andOther(primary.getTrigger(DIGITAL_R2)->negate())
           ->andOther(negatedHang)
           ->andOther(negatedLBLoad)
           ->whileTrue(liftSubsystem->positionCommand(200_deg, 0.0));
    primary.getTrigger(DIGITAL_L2)
           ->andOther(primary.getTrigger(DIGITAL_L1)->negate())
           ->andOther(primary.getTrigger(DIGITAL_R1)->negate())
           ->andOther(primary.getTrigger(DIGITAL_R2)->negate())
           ->andOther(negatedHang)
           ->andOther(negatedLBLoad)
           ->whileTrue(liftSubsystem->positionCommand(240_deg, 0.0));

    primary.getTrigger(DIGITAL_R2)
           ->andOther(primary.getTrigger(DIGITAL_R1)->negate())
           ->andOther(primary.getTrigger(DIGITAL_L1)->negate())
           ->andOther(primary.getTrigger(DIGITAL_L2)->negate())
           ->andOther(negatedHang)
           ->toggleOnTrue(intakeWithEject);
    primary.getTrigger(DIGITAL_R1)
           ->andOther(primary.getTrigger(DIGITAL_R2)->negate())
           ->andOther(primary.getTrigger(DIGITAL_L1)->negate())
           ->andOther(primary.getTrigger(DIGITAL_L2)->negate())
           ->andOther(negatedHang)
           ->toggleOnTrue(loadLB);

    primary.getTrigger(DIGITAL_LEFT)
    ->whileTrue(drivetrainSubsystem->characterizeAngular());
    primary.getTrigger(DIGITAL_UP)
           ->whileTrue(drivetrainSubsystem->characterizeLinear());

    auto* compTrigger = new Trigger([]() { return pros::c::competition_is_field(); });

    primary.getTrigger(DIGITAL_DOWN)->whileTrue((new Sequence({
            drivetrainSubsystem->pct(0.4, 0.4)->withTimeout(100_ms),
            drivetrainSubsystem->pct(0.2, 0.2)->with(new ScheduleCommand(cornerClearIntakeSequence))->withTimeout(600_ms),
            drivetrainSubsystem->pct(0.2, 0.2)->withTimeout(300_ms),
            drivetrainSubsystem->pct(-0.2, -0.2)->with(new ScheduleCommand(bottomIntakeSubsystem->pctCommand(1.0)->with(TopIntakePositionCommand::fromClosePositionCommand(topIntakeSubsystem, 0.95, 0.0))))->withTimeout(400_ms),
            drivetrainSubsystem->pct(0.0, 0.0)->withTimeout(300_ms),
        }))->repeatedly());
    primary.getTrigger(DIGITAL_RIGHT)->whileFalse(goalClampTrue);

    primary.getTrigger(DIGITAL_Y)
           ->andOther(negatedLBLoad)
            ->andOther(liftLow)
           ->andOther(new Trigger([]() { return !hangReleased; }))
           ->whileTrue(liftSubsystem->positionCommand(CONFIG::DESCORE_HEIGHT, 0.0)->race(TopIntakePositionCommand::fromClosePositionCommand(topIntakeSubsystem, -0.4, 0.0)));

    primary.getTrigger(DIGITAL_Y)
           ->andOther(negatedLBLoad)
            ->andOther(liftLow->negate())
           ->andOther(new Trigger([]() { return !hangReleased; }))
           ->whileTrue(liftSubsystem->positionCommand(CONFIG::DESCORE_HEIGHT, 0.0));

    primary.getTrigger(DIGITAL_Y)->andOther(new Trigger([]() { return hangReleased; }))->whileTrue(hang);

    primary.getTrigger(DIGITAL_B)->onTrue(
        liftSubsystem->positionCommand(10_deg)->withTimeout(1_s)->andThen(liftSubsystem->zero()));
}

inline void initializePathCommands()
{
    PathCommands::registerCommand("clamp", goalClampTrue);
    PathCommands::registerCommand("declamp", goalClampSubsystem->levelCommand(false));
    PathCommands::registerCommand("intakeWithEject", intakeWithEject);
    PathCommands::registerCommand("intakeNoEject", intakeNoEject);
    PathCommands::registerCommand("basicLoadLB", basicLoadLB);
    PathCommands::registerCommand("basicLoadLB2Ring",
                                  basicLoadLB->andThen(new ParallelCommandGroup(
                                      {
                                          topIntakeSubsystem->pctCommand(0.0), bottomIntakeSubsystem->pctCommand(1.0),
                                          liftSubsystem->positionCommand(CONFIG::WALL_STAKE_PRIME_HEIGHT, 0.0)
                                      })));
    PathCommands::registerCommand("loadLB", loadLB);
    PathCommands::registerCommand("bottomIntakeOffTopOn",
                                  topIntakeSubsystem->pctCommand(1.0)->with(bottomIntakeSubsystem->pctCommand(0.0)));
    PathCommands::registerCommand("stopIntake",
                                  bottomIntakeSubsystem->pctCommand(0.0)->with(
                                      TopIntakePositionCommand::fromClosePositionCommand(topIntakeSubsystem, 0.0)));
    PathCommands::registerCommand("stopIntake3",
                                  TopIntakePositionCommand::fromForwardPositionCommand(topIntakeSubsystem, 3.1, 0.0));
    PathCommands::registerCommand("hangRelease", hangRelease);
    PathCommands::registerCommand(
        "resetLB", liftSubsystem->positionCommand(10_deg)->withTimeout(1_s)->andThen(liftSubsystem->zero()));
    PathCommands::registerCommand("liftZero", liftSubsystem->positionCommand(6_deg));
    PathCommands::registerCommand("scoreAllianceStake", liftSubsystem->positionCommand(200_deg, 0.0));
    PathCommands::registerCommand("outtakeBottom", bottomIntakeSubsystem->pctCommand(-1.0));
    PathCommands::registerCommand("LBdrop", liftSubsystem->positionCommand(140_deg, 0.0));
    PathCommands::registerCommand("lbTouch", liftSubsystem->positionCommand(150_deg, 0.0));
}

inline void initializeCommands()
{
    goalClampTrue = goalClampSubsystem->levelCommand(true);

    intakeNoEject =
        new ParallelCommandGroup({
            bottomIntakeSubsystem->pctCommand(1.0), topIntakeSubsystem->pctCommand(1.0)
        });

    topIntakeWithEject =
        (new Sequence(
            {
                topIntakeSubsystem->pctCommand(1.0)->until([]()
                {
                    return static_cast<Alliance>(topIntakeSubsystem->getRing()) == OPPONENTS &&
                        std::fmod(std::fmod(topIntakeSubsystem->getPosition(), 1.0) + 10.0, 1.0) > 0.72;
                }),
                topIntakeSubsystem->pctCommand(1.0)->withTimeout(0.05_s),
                topIntakeSubsystem->pctCommand(1.0)->until([]()
                {
                    auto position = std::fmod(std::fmod(topIntakeSubsystem->getPosition(), 1.0) + 10.0, 1.0);
                    return position > 0.70 && position < 0.75;
                }),
                topIntakeSubsystem->pctCommand(-1.0)->withTimeout(0.07_s)
            }))
        ->repeatedly();

    intakeWithEject = topIntakeWithEject->with(bottomIntakeSubsystem->pctCommand(1.0));
    bottomOuttakeWithEject = topIntakeWithEject->with(bottomIntakeSubsystem->pctCommand(-1.0));
    cornerClearIntakeSequence = topIntakeWithEject->with(bottomIntakeSubsystem->pctCommand(1.0)->withTimeout(10_ms)->andThen(bottomIntakeSubsystem->pctCommand(-1.0)->withTimeout(200_ms)->andThen(bottomIntakeSubsystem->pctCommand(1.0)->withTimeout(390_ms))));

    if (topIntakeSubsystem->visionConnected())
    {
        basicLoadLB = new Sequence({
            new ParallelRaceGroup({
                bottomIntakeSubsystem->pctCommand(1.0),
                topIntakeSubsystem->pctCommand(0.0),
                liftSubsystem->positionCommand(CONFIG::LIFT_IDLE_POSITION, 35.0_deg),
            }),
            (new ParallelRaceGroup({
                intakeWithEject, liftSubsystem->positionCommand(CONFIG::LIFT_IDLE_POSITION, 0.0),
                new WaitUntilCommand([]()
                {
                    return static_cast<Alliance>(topIntakeSubsystem->getRing()) == ALLIANCE;
                })
            }))->until([]() { return topIntakeSubsystem->stalled(800_ms); }),
            new ParallelRaceGroup({
                bottomIntakeSubsystem->pctCommand(1.0),
                topIntakeSubsystem->pctCommand(1.0)->until([]() { return topIntakeSubsystem->stalled(200_ms); }),
                liftSubsystem->positionCommand(CONFIG::WALL_STAKE_LOAD_HEIGHT, 0.0),
            }),
            // new ParallelRaceGroup({bottomIntakeSubsystem->pctCommand(1.0), topIntakeSubsystem->pctCommand(0.0),
            //                        liftSubsystem->positionCommand(CONFIG::WALL_STAKE_LOAD_HEIGHT, 0.0),
            //                        new WaitCommand(50_ms)}),
            // new ParallelRaceGroup({bottomIntakeSubsystem->pctCommand(1.0), topIntakeSubsystem->pctCommand(1.0),
            //                        liftSubsystem->positionCommand(CONFIG::WALL_STAKE_LOAD_HEIGHT, 0.0),
            //                        new WaitCommand(80_ms)}),
            new ParallelRaceGroup({
                bottomIntakeSubsystem->pctCommand(1.0), topIntakeSubsystem->pctCommand(0.0),
                liftSubsystem->positionCommand(CONFIG::WALL_STAKE_PRIME_HEIGHT, 0.0),
                new WaitCommand(150_ms)
            }),
        });
    }
    else
    {
        basicLoadLB = new Sequence({
            new ParallelRaceGroup({
                bottomIntakeSubsystem->pctCommand(1.0),
                topIntakeSubsystem->pctCommand(0.0),
                liftSubsystem->positionCommand(CONFIG::LIFT_IDLE_POSITION, 35.0_deg),
            }),
            new ParallelRaceGroup({
                bottomIntakeSubsystem->pctCommand(1.0),
                topIntakeSubsystem->pctCommand(1.0)->until([]() { return topIntakeSubsystem->stalled(200_ms); }),
                liftSubsystem->positionCommand(CONFIG::WALL_STAKE_LOAD_HEIGHT, 0.0),
            }),
            // new ParallelRaceGroup({bottomIntakeSubsystem->pctCommand(1.0), topIntakeSubsystem->pctCommand(0.0),
            //                        liftSubsystem->positionCommand(CONFIG::WALL_STAKE_LOAD_HEIGHT, 0.0),
            //                        new WaitCommand(50_ms)}),
            // new ParallelRaceGroup({bottomIntakeSubsystem->pctCommand(1.0), topIntakeSubsystem->pctCommand(1.0),
            //                        liftSubsystem->positionCommand(CONFIG::WALL_STAKE_LOAD_HEIGHT, 0.0),
            //                        new WaitCommand(80_ms)}),
            new ParallelRaceGroup({
                bottomIntakeSubsystem->pctCommand(1.0), topIntakeSubsystem->pctCommand(0.0),
                liftSubsystem->positionCommand(CONFIG::WALL_STAKE_PRIME_HEIGHT, 0.0),
                new WaitCommand(150_ms)
            }),
        });
    }

    loadLB = new Sequence(
        {basicLoadLB, new ScheduleCommand(liftSubsystem->positionCommand(CONFIG::WALL_STAKE_PRIME_HEIGHT, 0.0))});

    letOutString =
        (new ParallelRaceGroup({
            drivetrainSubsystem->hangOut(1.0, -5.0_in), hangSubsystem->levelCommand(false),
            liftSubsystem->positionCommand(70_deg, 0.0)
        }))
        ->andThen(new ParallelRaceGroup({
            drivetrainSubsystem->hangOut(1.0, 7.5_in),
            hangSubsystem->levelCommand(false)
                         ->with(liftSubsystem->positionCommand(15_deg, 0.0))
                         ->until([]() { return drivetrainSubsystem->getStringDistance() > 4.2_in; })
                         ->andThen(hangSubsystem->levelCommand(true)->with(
                             liftSubsystem->positionCommand(70_deg, 0.0))),
        }));

    gripBar = new Sequence(
        {
            new ParallelRaceGroup({
                drivetrainSubsystem->hangIn(1.0, -6.9_in), hangSubsystem->levelCommand(true),
                liftSubsystem->positionCommand(70_deg, 0.0)
            }),
            new ParallelRaceGroup({
                drivetrainSubsystem->hangIn(1.0, -7.4_in), hangSubsystem->levelCommand(false),
                liftSubsystem->positionCommand(70_deg, 0.0)
            })
        });

    barToBarHang = new Sequence({letOutString, gripBar});

    hang = new ParallelRaceGroup(
        {
            new Sequence({
                drivetrainSubsystem->activatePto(),
                drivetrainSubsystem->pct(-0.1, -0.1)->withTimeout(70_ms),
                drivetrainSubsystem->pct(0.1, 0.1)->withTimeout(70_ms),
                gripBar,
                barToBarHang,
                barToBarHang,
                (new ParallelRaceGroup({
                    drivetrainSubsystem->hangOut(1.0, -5.0_in), hangSubsystem->levelCommand(false),
                    liftSubsystem->positionCommand(70_deg, 0.0)
                })),
            }),
            topIntakeSubsystem->pctCommand(0.0), bottomIntakeSubsystem->pctCommand(0.0)
        });

    hangRelease =
        new ParallelCommandGroup({
            hangSubsystem->levelCommand(true), liftSubsystem->positionCommand(90_deg, 0.0),
            topIntakeSubsystem->pctCommand(0.0), bottomIntakeSubsystem->pctCommand(0.0)
        });

    hangIdle = new ParallelCommandGroup({
                hangSubsystem->levelCommand(false),
                liftSubsystem->positionCommand(30_deg, 0.0),
                topIntakeSubsystem->pctCommand(0.0),
                bottomIntakeSubsystem->pctCommand(0.0)
    });
}

inline void subsystemInit()
{
    TELEMETRY.setSerial(new pros::Serial(0, 921600));

    // 'd' is hang - done
    // 'a' is hang - done
    // 'e' is back clamp - done
    // 'c' is pto - done
    // 'b' is potentiometer - done
    // 5 is inertial
    // 6 is top intake motor
    // 3 is rotation on odom
    // 2 is right distance sensor
    // 9 is back distance sensor
    // 19 is left distance sensor
    // 20 is the front distance sensor
    // 15 is lb oppisite of last bot
    // 12 is bottom intake
    // 4 is back right drive motor - reversed
    // 7 is middle drive mtoro right
    // 10 is front drive motor right
    // 16 is back left drive motor - reversed
    // 17 is front drive motor left
    // 18 middle drive motor left

    topIntakeSubsystem = new TopIntakeSubsystem({6}, pros::Optical(21));
    bottomIntakeSubsystem = new MotorSubsystem(pros::Motor(-12));
    liftSubsystem = new LiftSubsystem({-15}, PID(1.2, 0.0, 3.0, 0.2, 1.0));
    goalClampSubsystem = new SolenoidSubsystem(pros::adi::DigitalOut('e'));
    hangSubsystem = new SolenoidSubsystem({pros::adi::DigitalOut('a'), pros::adi::DigitalOut('d')});
    drivetrainSubsystem = new DrivetrainSubsystem(
        {16, -17, -18}, {-4, 7, 10}, pros::Imu(5), pros::adi::DigitalOut('c'), pros::Rotation(-8),
        []() { return goalClampSubsystem->getLastValue(); },
        pros::Rotation(3)); // wheels listed back to front; 8 for rotation sensor on pto

    pros::Task([]()
    {
        if (!topIntakeSubsystem->visionConnected())
        {
            primary.rumble(".. --");
            pros::delay(2000);
        }

        if (!drivetrainSubsystem->odomConnected())
        {
            primary.rumble("-- --");
            pros::delay(2000);
        }

        if (pros::battery::get_capacity() < 50.0)
        {
            primary.rumble("..-");
            pros::delay(2000);
        }

        // Check motor temps
        if (std::max({
            drivetrainSubsystem->getTopMotorTemp(), topIntakeSubsystem->getTopMotorTemp(),
            bottomIntakeSubsystem->getTopMotorTemp(), liftSubsystem->getTopMotorTemp()
        }) >= 45.0)
        {
            primary.rumble(".--");
        }
    });

    drivetrainSubsystem->addLocalizationSensor(new Distance(CONFIG::DISTANCE_LEFT_OFFSET, 0.987, pros::Distance(19)));
    drivetrainSubsystem->addLocalizationSensor(new Distance(CONFIG::DISTANCE_FRONT_OFFSET, 0.986, pros::Distance(20)));
    drivetrainSubsystem->addLocalizationSensor(new Distance(CONFIG::DISTANCE_RIGHT_OFFSET, 0.980, pros::Distance(2)));
    drivetrainSubsystem->addLocalizationSensor(new Distance(CONFIG::DISTANCE_BACK_OFFSET, 0.979, pros::Distance(9)));

    drivetrainSubsystem->initUniform(-70_in, -70_in, 70_in, 70_in, 0_deg, false);

    CommandScheduler::registerSubsystem(drivetrainSubsystem, drivetrainSubsystem->tank(primary));
    CommandScheduler::registerSubsystem(
        topIntakeSubsystem, TopIntakePositionCommand::fromClosePositionCommand(topIntakeSubsystem, 0.0, 0.0));
    CommandScheduler::registerSubsystem(bottomIntakeSubsystem, bottomIntakeSubsystem->stopIntake());
    CommandScheduler::registerSubsystem(liftSubsystem, liftSubsystem->positionCommand(CONFIG::LIFT_IDLE_POSITION, 0.0));
    CommandScheduler::registerSubsystem(goalClampSubsystem, goalClampSubsystem->levelCommand(false));
    CommandScheduler::registerSubsystem(hangSubsystem, hangSubsystem->levelCommand(false));

    initializeCommands();
    initializeController();
    initializePathCommands();
}
