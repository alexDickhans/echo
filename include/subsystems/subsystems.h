#pragma once

#include "command/commandScheduler.h"
#include "lift.h"
#include "topIntake.h"
#include "goalClamp.h"
#include "bottomIntake.h"
#include "drivetrain.h"
#include "command/commandController.h"
#include "command/parallelCommandGroup.h"
#include "command/parallelRaceGroup.h"
#include "command/sequence.h"
#include "command/waitUntilCommand.h"
#include "hook.h"
#include "command/repeatCommand.h"
#include "command/waitCommand.h"
#include "commands/ramsete.h"
#include "commands/driveMove.h"
#include "command/conditionalCommand.h"
#include "commands/rotate.h"
#include "localization/line.h"
#include "localization/distance.h"
#include "localization/gps.h"
#include "commands/driveToGoal.h"
#include "command/scheduleCommand.h"
#include "pros/adi.hpp"
#include "motionProfiling/pathCommands.h"
#include "command/proxyCommand.h"

Drivetrain *drivetrain;
TopIntake *topIntake;
BottomIntake *bottomIntake;
LiftSubsystem *lift;
GoalClamp *goalClamp;
Hook *hook;

Command *loadOneRingHigh;
Command *loadOneRingLow;
Command *intakeOntoGoal;

Command *goalClampTrue;

CommandController primary(pros::controller_id_e_t::E_CONTROLLER_MASTER);
CommandController partner(pros::controller_id_e_t::E_CONTROLLER_PARTNER);

pros::Distance intakeDistance(21);
pros::Distance goalClampDistanceSensor(20);

GpsSensor* gpsSensor;

bool outtakeWallStake = false;
bool hasRings = false;

inline void subsystemInit() {
	TELEMETRY.setSerial(new pros::Serial(19, 921600));

	drivetrain = new Drivetrain({-2, -3}, {6, 7}, {4}, {-9}, pros::Imu(16));
	topIntake = new TopIntake(pros::Motor(-5));
	bottomIntake = new BottomIntake(pros::Motor(-10));
	lift = new LiftSubsystem(pros::Motor(1), PID(30.0, 0.0, 50.0));
	goalClamp = new GoalClamp(pros::adi::DigitalOut('a'));
	hook = new Hook(pros::Motor(-8));

	// drivetrain->addLocalizationSensor(new LineSensor(CONFIG::LINE_SENSOR_1_OFFSET, pros::adi::LineSensor('b')));
	drivetrain->addLocalizationSensor(new Distance(CONFIG::DISTANCE_LEFT_OFFSET, pros::Distance(15)));
	drivetrain->addLocalizationSensor(new Distance(CONFIG::DISTANCE_BACK_OFFSET, pros::Distance(14)));
	drivetrain->addLocalizationSensor(new Distance(CONFIG::DISTANCE_RIGHT_OFFSET, pros::Distance(11)));
	// drivetrain->addLocalizationSensor(new GpsSensor(CONFIG::GPS_OFFSET.z(),
	// 						pros::Gps(12, -CONFIG::GPS_OFFSET.y(), CONFIG::GPS_OFFSET.x())));

	drivetrain->initUniform(-70_in, -70_in, 70_in, 70_in, 0_deg, false);

	// drivetrain->initNorm(Eigen::Vector2f::Constant(0.0), Eigen::Matrix2f::Identity() * 0.01, 0.0, false);

	CommandScheduler::registerSubsystem(drivetrain, drivetrain->tank(primary));
	CommandScheduler::registerSubsystem(
		topIntake, new ConditionalCommand(topIntake->stopIntake(), topIntake->positionCommandFwd(0.0),
		                                  [&]() { return hasRings; }));
	CommandScheduler::registerSubsystem(bottomIntake, bottomIntake->stopIntake());
	CommandScheduler::registerSubsystem(lift, new ConditionalCommand(lift->positionCommand(3.0_deg),
	                                                                 lift->positionCommand(0.0_deg),
	                                                                 [&]() { return hasRings; }));
	CommandScheduler::registerSubsystem(goalClamp, goalClamp->levelCommand(false));
	CommandScheduler::registerSubsystem(hook, hook->positionCommand(0.0));

	goalClampTrue = goalClamp->levelCommand(true);

	loadOneRingLow = new Sequence({
		new ParallelRaceGroup({
			bottomIntake->movePct(1.0),
			lift->positionCommand(0.0_deg),
			topIntake->positionCommandFwd(0.3),
			new WaitUntilCommand([&]() { return intakeDistance.get() < 100; })
		}),
		new ParallelRaceGroup({
			bottomIntake->movePct(1.0),
			lift->positionCommand(0.0_deg),
			new ParallelCommandGroup({topIntake->moveToPositionFwd(1.3), new WaitCommand(0.5_s)}),
		}),
	});
	loadOneRingHigh = new Sequence({
		new ParallelRaceGroup({
			bottomIntake->movePct(1.0),
			lift->positionCommand(6.0_deg),
			topIntake->positionCommandRwd(-0.1),
			new WaitUntilCommand([&]() { return intakeDistance.get() < 100; })
		}),
		new ParallelRaceGroup({
			bottomIntake->movePct(1.0),
			lift->positionCommand(6.0_deg),
			new ParallelCommandGroup({topIntake->moveToPositionRwd(-1.1), new WaitCommand(0.5_s)}),
		}),
	});
	intakeOntoGoal = new ParallelCommandGroup({
		bottomIntake->movePct(1.0),
		lift->positionCommand(0.0_deg),
		topIntake->movePct(1.0),
		new InstantCommand([&]() { hasRings = false; }, {}),
	});

	primary.getTrigger(DIGITAL_X)->toggleOnTrue(drivetrain->arcade(primary));

	primary.getTrigger(DIGITAL_L1)->toggleOnTrue(
		new Sequence({
			new InstantCommand([&]() { hasRings = true; }, {}),
			new Sequence({
				loadOneRingHigh,
				loadOneRingHigh
			})
		})
	);

	primary.getTrigger(DIGITAL_L2)->toggleOnTrue(
		new Sequence({
			new InstantCommand([&]() { hasRings = false; }, {}),
			loadOneRingLow,
			loadOneRingLow
		})
	);

	primary.getTrigger(DIGITAL_R2)->toggleOnTrue(intakeOntoGoal);
	primary.getTrigger(DIGITAL_R1)->onTrue(new Sequence({
		new InstantCommand([&]() { outtakeWallStake = false; }, {}),
		new ParallelRaceGroup({
			bottomIntake->movePct(0.0),
			lift->moveToPosition(33_deg, 0.3_deg),
			topIntake->movePct(0.0),
			hook->positionCommand(5_deg),
		}),
		new ParallelRaceGroup({
			bottomIntake->movePct(0.0),
			lift->positionCommand(33_deg),
			topIntake->movePct(0.0),
			hook->positionCommand(0_deg),
			new WaitUntilCommand([&]() { return primary.get_digital(DIGITAL_Y); })
		}),
		new InstantCommand([&]() {
			outtakeWallStake = true;
			hasRings = false;
		}, {}),
		new ParallelCommandGroup({
			bottomIntake->movePct(0.0),
			lift->positionCommand(33_deg),
			topIntake->movePct(-1.0),
			hook->positionCommand(0_deg),
		})
	}))->onFalse(new ParallelRaceGroup({
		bottomIntake->movePct(0.0),
		lift->moveToPosition(0_deg, 10_deg),
		new ConditionalCommand(topIntake->movePct(1.0), topIntake->movePct(0.0), [&]() { return outtakeWallStake; }),
		hook->positionCommand(5_deg),
	}));

	primary.getTrigger(DIGITAL_RIGHT)->toggleOnTrue(goalClampTrue);

	primary.getTrigger(DIGITAL_B)->whileTrue(new Sequence({
		new ScheduleCommand(goalClamp->levelCommand(false)),
		(new DriveToGoal(drivetrain, CONFIG::GOAL_PID, -0.6))->until([&]() { return goalClampDistanceSensor.get_distance() < 10; })->withTimeout(2_s),
		new ScheduleCommand(goalClampTrue),
	}));

	partner.getTrigger(DIGITAL_A)->whileTrue(new ParallelCommandGroup({
		new InstantCommand([&]() { hasRings = false; }, {}), bottomIntake->movePct(0.8), lift->positionCommand(8.0_deg),
		topIntake->movePct(-1.0)
	}));

	partner.getTrigger(DIGITAL_UP)->whileTrue(hook->positionCommand(0.37));
	partner.getTrigger(DIGITAL_LEFT)->whileTrue(hook->positionCommand(0.44));
	partner.getTrigger(DIGITAL_RIGHT)->whileTrue(hook->positionCommand(0.48));
	partner.getTrigger(DIGITAL_DOWN)->whileTrue(hook->positionCommand(0.58));
	partner.getTrigger(DIGITAL_L2)->whileTrue(lift->controller(&partner));
	Trigger([&]() { return abs(partner.get_analog(ANALOG_RIGHT_Y)) > 15; }, CommandScheduler::getTeleopEventLoop()).
			whileTrue(topIntake->controller(&partner));
	partner.getTrigger(DIGITAL_X)->whileTrue(bottomIntake->movePct(1.0));
	partner.getTrigger(DIGITAL_B)->whileTrue(bottomIntake->movePct(-1.0));
	partner.getTrigger(DIGITAL_R1)->whileTrue(new Sequence({
		new ParallelRaceGroup({
			bottomIntake->movePct(0.0),
			lift->moveToPosition(7_deg, 0.3_deg),
			topIntake->movePct(0.0),
			hook->positionCommand(5_deg),
		}),
		new ParallelRaceGroup({
			bottomIntake->movePct(0.0),
			lift->positionCommand(7_deg),
			topIntake->movePct(0.0),
			hook->positionCommand(5_deg),
			new WaitUntilCommand([&]() { return partner.get_digital(DIGITAL_R2); })
		}),
		new ParallelCommandGroup({
			bottomIntake->movePct(0.0),
			lift->positionCommand(0_deg),
			topIntake->movePct(1.0),
			hook->positionCommand(0_deg),
		})
	}));

	PathCommands::registerCommand("intakeNeutralStakes", loadOneRingHigh->andThen(loadOneRingHigh));
	PathCommands::registerCommand("intakeAllianceStakes", loadOneRingLow->andThen(loadOneRingLow));
	PathCommands::registerCommand("scoreNeutral", new Sequence(
			{
			 new ParallelRaceGroup({
				 bottomIntake->movePct(0.0),
				 lift->moveToPosition(33_deg, 0.3_deg),
				 topIntake->movePct(0.0),
				 hook->positionCommand(5_deg),
			 }),
			 new ParallelRaceGroup({bottomIntake->movePct(0.0), lift->moveToPosition(33_deg, 1_deg),
						topIntake->movePct(0.0), hook->positionCommand(0_deg),}),
			 new ParallelCommandGroup({
				 bottomIntake->movePct(0.0),
				 lift->positionCommand(33_deg),
				 topIntake->movePct(-1.0),
				 hook->positionCommand(0_deg),
			 })}));
	PathCommands::registerCommand("intakeGoal", new ParallelCommandGroup({topIntake->movePct(1.0), bottomIntake->movePct(1.0), lift->positionCommand(0.0)}));
	PathCommands::registerCommand("clamp", goalClamp->levelCommand(true));
	PathCommands::registerCommand("declamp", goalClamp->levelCommand(false));
}
