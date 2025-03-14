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
#include "commands/lift/trapPosition.h"

#include <queue>
#include <algorithm>

inline DrivetrainSubsystem *drivetrainSubsystem;
inline TopIntakeSubsystem *topIntakeSubsystem;
inline MotorSubsystem *bottomIntakeSubsystem;
inline LiftSubsystem *liftSubsystem;
inline SolenoidSubsystem *goalClampSubsystem;
inline SolenoidSubsystem *hangSubsystem;
inline SolenoidSubsystem *doinker;

inline Command *loadLB;
inline Command *intakeWithEject;
inline Command *goalClampTrue;
inline Command *hang;
inline Command *barToBarHang;
inline Command *gripBar;
inline Command *letOutString;
inline Command *basicLoadLB;
inline Command *fastLoadLB;
inline Command *intakeNoEject;
inline Command *hangRelease;

inline Command *doinkerDown;

inline CommandController primary(pros::controller_id_e_t::E_CONTROLLER_MASTER);
inline CommandController partner(pros::controller_id_e_t::E_CONTROLLER_PARTNER);

inline Trigger *negatedHang;
inline Trigger *negatedLBLoad;

bool hangReleased = false;

inline void initializeController() {
    primary.getTrigger(DIGITAL_X)->toggleOnTrue(drivetrainSubsystem->arcade(primary));
    primary.getTrigger(DIGITAL_A)->whileTrue(bottomIntakeSubsystem->pctCommand(-1.0));

    primary.getTrigger(DIGITAL_R1)->andOther(primary.getTrigger(DIGITAL_L1))->andOther(primary.getTrigger(DIGITAL_R2))->
            andOther(primary.getTrigger(DIGITAL_L2))->onTrue(
                (new InstantCommand([]() { hangReleased = true; }, {}))->andThen(hangRelease));

    negatedHang = (new Trigger(hangRelease))->negate();
    negatedLBLoad = (new Trigger(loadLB))->negate();

    primary.getTrigger(DIGITAL_L1)->andOther(primary.getTrigger(DIGITAL_L2)->negate())->
            andOther(primary.getTrigger(DIGITAL_R1)->negate())->andOther(primary.getTrigger(DIGITAL_R2)->negate())->
            andOther(negatedHang)->andOther(negatedLBLoad)->
            whileTrue(
                liftSubsystem->positionCommand(200_deg, 0.0));
    primary.getTrigger(DIGITAL_L2)->andOther(primary.getTrigger(DIGITAL_L1)->negate())->
            andOther(primary.getTrigger(DIGITAL_R1)->negate())->andOther(primary.getTrigger(DIGITAL_R2)->negate())->
            andOther(negatedHang)->andOther(negatedLBLoad)->
            whileTrue(liftSubsystem->positionCommand(240_deg, 0.0));

    primary.getTrigger(DIGITAL_R2)->andOther(primary.getTrigger(DIGITAL_R1)->negate())->
            andOther(primary.getTrigger(DIGITAL_L1)->negate())->andOther(primary.getTrigger(DIGITAL_L2)->negate())->
            andOther(negatedHang)->
            toggleOnTrue(intakeWithEject);
    primary.getTrigger(DIGITAL_R1)->andOther(primary.getTrigger(DIGITAL_R2)->negate())->
            andOther(primary.getTrigger(DIGITAL_L1)->negate())->andOther(primary.getTrigger(DIGITAL_L2)->negate())->
            andOther(negatedHang)->
            toggleOnTrue(loadLB);

    primary.getTrigger(DIGITAL_UP)->toggleOnTrue(doinkerDown);
    primary.getTrigger(DIGITAL_DOWN)->onTrue(topIntakeSubsystem->pctCommand(-1.0));

    primary.getTrigger(DIGITAL_RIGHT)->whileFalse(goalClampTrue);
    primary.getTrigger(DIGITAL_Y)->andOther(negatedLBLoad)->andOther(new Trigger([]() { return !hangReleased; }))
            ->whileTrue(liftSubsystem->positionCommand(140_deg, 0.0));
    primary.getTrigger(DIGITAL_Y)->andOther(new Trigger([]() { return hangReleased; }))
            ->whileTrue(hang);

    primary.getTrigger(DIGITAL_B)->onTrue(
        liftSubsystem->positionCommand(10_deg)->withTimeout(1_s)->andThen(liftSubsystem->zero()));
}

inline void initializePathCommands() {
    PathCommands::registerCommand("clamp", goalClampTrue);
    PathCommands::registerCommand("declamp", goalClampSubsystem->levelCommand(false));
    PathCommands::registerCommand("intakeWithEject", intakeWithEject);
    PathCommands::registerCommand("intakeNoEject", intakeNoEject);
    PathCommands::registerCommand("basicLoadLB", basicLoadLB);
    PathCommands::registerCommand("basicLoadLB2Ring", basicLoadLB->andThen(new ParallelCommandGroup({
                                      topIntakeSubsystem->pctCommand(0.0), bottomIntakeSubsystem->pctCommand(1.0),
                                      liftSubsystem->positionCommand(CONFIG::WALL_STAKE_PRIME_HEIGHT, 0.0)
                                  })));
    PathCommands::registerCommand("loadLB", loadLB);
    PathCommands::registerCommand("bottomIntakeOffTopOn",
                                  topIntakeSubsystem->pctCommand(1.0)->with(bottomIntakeSubsystem->pctCommand(0.0)));
    PathCommands::registerCommand("stopIntake",
                                  bottomIntakeSubsystem->pctCommand(0.0)->with(TopIntakePositionCommand::fromClosePositionCommand(topIntakeSubsystem, 0.0)));
    PathCommands::registerCommand("stopIntake3",
                                  TopIntakePositionCommand::fromForwardPositionCommand(topIntakeSubsystem, 2.20, 0.0));
    PathCommands::registerCommand("hangRelease", hangRelease);
    PathCommands::registerCommand(
        "resetLB", liftSubsystem->positionCommand(10_deg)->withTimeout(1_s)->andThen(liftSubsystem->zero()));
    PathCommands::registerCommand("liftZero", liftSubsystem->positionCommand(6_deg));
    PathCommands::registerCommand("scoreAllianceStake", liftSubsystem->positionCommand(185_deg, 0.0));
    PathCommands::registerCommand("outtakeBottom", bottomIntakeSubsystem->pctCommand(-1.0));
    PathCommands::registerCommand("fastLoadLB", fastLoadLB);
    PathCommands::registerCommand("doinkerDown", doinkerDown);
    PathCommands::registerCommand("doinkerUp", doinker->levelCommand(true));
    PathCommands::registerCommand("LBdrop", liftSubsystem->positionCommand(120_deg, 0.0));
}

inline void initializeCommands() {
    goalClampTrue = goalClampSubsystem->levelCommand(true);

    intakeNoEject = new ParallelCommandGroup({
        new InstantCommand([] () { std::cout << "Starting intake command" << std::endl; }, {}),
        bottomIntakeSubsystem->pctCommand(1.0), topIntakeSubsystem->pctCommand(1.0)
    });

    intakeWithEject = (new Sequence({
        intakeNoEject->until([]() { return static_cast<Alliance>(topIntakeSubsystem->getRing()) == OPPONENTS; }),
        intakeNoEject->until([]() {
            auto position = std::fmod(std::fmod(topIntakeSubsystem->getPosition(), 1.0) + 10.0, 1.0);
            return position > 0.365 && position < 0.45; // tune these variables to make ejection work better
        }),
        bottomIntakeSubsystem->pctCommand(1.0)->race(topIntakeSubsystem->pctCommand(-1.0)->withTimeout(0.03_s))
    }))->repeatedly();

    basicLoadLB = new Sequence({
        new ParallelRaceGroup({
            bottomIntakeSubsystem->pctCommand(1.0),
            topIntakeSubsystem->pctCommand(0.0),
            liftSubsystem->positionCommand(CONFIG::LIFT_IDLE_POSITION, 35.0_deg),
        }),
        new ParallelRaceGroup({
            intakeWithEject,
            liftSubsystem->positionCommand(CONFIG::LIFT_IDLE_POSITION, 0.0),
            new WaitUntilCommand([]() { return static_cast<Alliance>(topIntakeSubsystem->getRing()) == ALLIANCE; })
        }),
        new ParallelRaceGroup({
            bottomIntakeSubsystem->pctCommand(1.0),
            topIntakeSubsystem->pctCommand(1.0)->until([]() { return topIntakeSubsystem->stalled(300_ms); }),
            liftSubsystem->positionCommand(CONFIG::WALL_STAKE_LOAD_HEIGHT, 0.0),
        }),
        new ParallelRaceGroup({
            bottomIntakeSubsystem->pctCommand(1.0), topIntakeSubsystem->pctCommand(-1.0),
            liftSubsystem->positionCommand(CONFIG::WALL_STAKE_LOAD_HEIGHT, 0.0), new WaitCommand(50_ms)
        }),
        new ParallelRaceGroup({
            bottomIntakeSubsystem->pctCommand(1.0), topIntakeSubsystem->pctCommand(1.0),
            liftSubsystem->positionCommand(CONFIG::WALL_STAKE_LOAD_HEIGHT, 0.0), new WaitCommand(130_ms)
        }),
        new ParallelRaceGroup({
            bottomIntakeSubsystem->pctCommand(1.0), topIntakeSubsystem->pctCommand(0.0),
            liftSubsystem->positionCommand(CONFIG::WALL_STAKE_PRIME_HEIGHT, 0.0), new WaitCommand(300_ms)
        }),
    });

    fastLoadLB = new Sequence({
        new ParallelRaceGroup({
            bottomIntakeSubsystem->pctCommand(1.0),
            topIntakeSubsystem->pctCommand(1.0)->until([]() { return topIntakeSubsystem->stalled(300_ms); }),
            liftSubsystem->positionCommand(CONFIG::WALL_STAKE_LOAD_HEIGHT - 1_deg, 0.0),
        }),
        new ParallelRaceGroup({
            bottomIntakeSubsystem->pctCommand(1.0), topIntakeSubsystem->pctCommand(-1.0),
            liftSubsystem->positionCommand(CONFIG::WALL_STAKE_LOAD_HEIGHT - 1_deg, 0.0), new WaitCommand(90_ms)
        }),
        new ParallelRaceGroup({
            bottomIntakeSubsystem->pctCommand(1.0), topIntakeSubsystem->pctCommand(1.0),
            liftSubsystem->positionCommand(CONFIG::WALL_STAKE_LOAD_HEIGHT - 1_deg, 0.0), new WaitCommand(160_ms)
        }),
        new ParallelRaceGroup({
            bottomIntakeSubsystem->pctCommand(1.0), topIntakeSubsystem->pctCommand(0.0),
            liftSubsystem->positionCommand(CONFIG::WALL_STAKE_PRIME_HEIGHT, 0.0), new WaitCommand(400_ms)
        }),
    });

    loadLB = new Sequence({
        basicLoadLB,
        new ScheduleCommand(liftSubsystem->positionCommand(CONFIG::WALL_STAKE_PRIME_HEIGHT, 0.0))
    });

    letOutString = (new ParallelRaceGroup({
        drivetrainSubsystem->hangOut(1.0, -5.0_in),
        hangSubsystem->levelCommand(false),
        liftSubsystem->positionCommand(70_deg, 0.0)
    }))->andThen(new ParallelRaceGroup({
        drivetrainSubsystem->hangOut(1.0, 6.0_in),
        hangSubsystem->levelCommand(false)->with(liftSubsystem->positionCommand(15_deg, 0.0))->until([]() {
            return drivetrainSubsystem->getStringDistance() > 3.0_in;
        })->andThen(hangSubsystem->levelCommand(true)->with(liftSubsystem->positionCommand(70_deg, 0.0))),
    }));

    gripBar = new Sequence({
        new ParallelRaceGroup({
            drivetrainSubsystem->hangIn(1.0, -7.8_in),
            hangSubsystem->levelCommand(true),
            liftSubsystem->positionCommand(70_deg, 0.0)
        }),
        new ParallelRaceGroup({
            drivetrainSubsystem->hangPctCommand(0.03),
            hangSubsystem->levelCommand(false),
            liftSubsystem->positionCommand(70_deg, 0.0),
            new WaitCommand(200_ms),
        })
    });

    barToBarHang =
            new Sequence({
                letOutString,
                gripBar
            });

    hang = new ParallelCommandGroup({
        new Sequence({
            drivetrainSubsystem->activatePto(),
            drivetrainSubsystem->pct(-0.1, -0.1)->withTimeout(70_ms),
            drivetrainSubsystem->pct(0.1, 0.1)->withTimeout(70_ms),
            gripBar,
            barToBarHang,
            barToBarHang,
            drivetrainSubsystem->pct(0.0, 0.0),
        }),
        topIntakeSubsystem->pctCommand(0.0), bottomIntakeSubsystem->pctCommand(0.0)
    });

    hangRelease =
            new ParallelCommandGroup({
                hangSubsystem->levelCommand(true),
                liftSubsystem->positionCommand(90_deg, 0.0),
                topIntakeSubsystem->pctCommand(0.0),
                bottomIntakeSubsystem->pctCommand(0.0)
            });

    doinkerDown = doinker->levelCommand(false);
}

inline void subsystemInit() {
    TELEMETRY.setSerial(new pros::Serial(0, 921600));

    topIntakeSubsystem = new TopIntakeSubsystem({3}, pros::AIVision(16));
    bottomIntakeSubsystem = new MotorSubsystem(pros::Motor(-2));
    liftSubsystem = new LiftSubsystem({-1}, PID(2.3, 0.0, 9.8, 0.2, 1.0));
    goalClampSubsystem = new SolenoidSubsystem(pros::adi::DigitalOut('c'));
    hangSubsystem = new SolenoidSubsystem({pros::adi::DigitalOut('d'), pros::adi::DigitalOut('b')});
    // 'd' left, 'b' right
    doinker = new SolenoidSubsystem(pros::adi::DigitalOut('e', true));
    drivetrainSubsystem = new DrivetrainSubsystem({-11, 13, -14}, {17, -19, 18}, pros::Imu(9),
                                                  pros::adi::DigitalOut('a'), pros::Rotation(-8), []() {
                                                      return goalClampSubsystem->getLastValue();
                                                  }); // wheels listed back to front; 8 for rotation sensor on pto

    pros::Task([]() {
        if (pros::battery::get_capacity() < 50.0) {
            primary.rumble("..-");
            pros::delay(2000);
        }

        // Check motor temps
        if (std::max({
                drivetrainSubsystem->getTopMotorTemp(),
                topIntakeSubsystem->getTopMotorTemp(),
                bottomIntakeSubsystem->getTopMotorTemp(),
                liftSubsystem->getTopMotorTemp()
            }) >= 45.0) {
            primary.rumble(".--");
        }
    });

    drivetrainSubsystem->addLocalizationSensor(new Distance(CONFIG::DISTANCE_LEFT_OFFSET, 2388.0 / 2445.0,
                                                            pros::Distance(4)));
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
    CommandScheduler::registerSubsystem(liftSubsystem, liftSubsystem->positionCommand(CONFIG::LIFT_IDLE_POSITION, 0.0));
    CommandScheduler::registerSubsystem(goalClampSubsystem, goalClampSubsystem->levelCommand(false));
    CommandScheduler::registerSubsystem(hangSubsystem, hangSubsystem->levelCommand(false));
    CommandScheduler::registerSubsystem(doinker, doinker->levelCommand(true));

    initializeCommands();
    initializeController();
    initializePathCommands();
}
