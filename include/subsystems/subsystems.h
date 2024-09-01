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

Drivetrain *drivetrain;
TopIntake *topIntake;
BottomIntake *bottomIntake;
LiftSubsystem *lift;
GoalClamp *goalClamp;
Hook *hook;

Command *loadOneRingHigh;
Command *loadOneRingLow;
Command *intakeOntoGoal;

CommandController primary(pros::controller_id_e_t::E_CONTROLLER_MASTER);
CommandController partner(pros::controller_id_e_t::E_CONTROLLER_PARTNER);

pros::Distance intakeDistance(21);

bool outtakeWallStake = false;

inline void subsystemInit() {
	drivetrain = new Drivetrain({-2, -3}, {6, 7}, {4}, {-9}, pros::Imu(16)); // TODO: actual ports
	topIntake = new TopIntake(pros::Motor(-5));
	bottomIntake = new BottomIntake(pros::Motor(-10));
	lift = new LiftSubsystem(pros::Motor(1), PID(30.0, 0.0, 50.0));
	goalClamp = new GoalClamp(pros::adi::DigitalOut('a'));
	hook = new Hook(pros::Motor(-8));

	CommandScheduler::registerSubsystem(drivetrain, drivetrain->tank(primary));
	CommandScheduler::registerSubsystem(topIntake, topIntake->stopIntake());
	CommandScheduler::registerSubsystem(bottomIntake, bottomIntake->stopIntake());
	CommandScheduler::registerSubsystem(lift, lift->positionCommand(3.0_deg));
	CommandScheduler::registerSubsystem(goalClamp, goalClamp->levelCommand(false));
	CommandScheduler::registerSubsystem(hook, hook->positionCommand(0.0));

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
		topIntake->movePct(1.0)
	});

	primary.getTrigger(DIGITAL_X)->toggleOnTrue(drivetrain->arcade(primary));

	primary.getTrigger(DIGITAL_L1)->toggleOnTrue(
		new Sequence({
			loadOneRingHigh,
			loadOneRingHigh
		})
	);

	primary.getTrigger(DIGITAL_L2)->toggleOnTrue(
	new Sequence({
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
			hook->positionCommand(5_deg),
			new WaitUntilCommand([&]() { return primary.get_digital(DIGITAL_Y); })
		}),
		new InstantCommand([&]() { outtakeWallStake = true; }, {}),
		new ParallelCommandGroup({
			bottomIntake->movePct(0.0),
			lift->positionCommand(33_deg),
			topIntake->movePct(-1.0)
		})
	}))->onFalse(new ParallelRaceGroup({
		bottomIntake->movePct(0.0),
		lift->moveToPosition(0_deg, 10_deg),
		new ConditionalCommand(topIntake->movePct(1.0), topIntake->movePct(0.0), [&]() { return outtakeWallStake; }),
		hook->positionCommand(5_deg),
	}));

	primary.getTrigger(DIGITAL_RIGHT)->toggleOnTrue(goalClamp->levelCommand(true));

	partner.getTrigger(DIGITAL_A)->whileTrue(new ParallelCommandGroup({
		bottomIntake->movePct(0.8), lift->positionCommand(8.0_deg), topIntake->movePct(-1.0)
	}));

	partner.getTrigger(DIGITAL_UP)->whileTrue(hook->positionCommand(0.37));
	partner.getTrigger(DIGITAL_LEFT)->whileTrue(hook->positionCommand(0.44));
	partner.getTrigger(DIGITAL_RIGHT)->whileTrue(hook->positionCommand(0.48));
	partner.getTrigger(DIGITAL_DOWN)->whileTrue(hook->positionCommand(0.58));
}
