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

Drivetrain *drivetrain;
TopIntake *topIntake;
BottomIntake *bottomIntake;
LiftSubsystem *lift;
GoalClamp *goalClamp;
Hook *hook;

Command* loadOneRingHigh;
Command* loadOneRingLow;
Command* intakeOntoGoal;

CommandController primary(pros::controller_id_e_t::E_CONTROLLER_MASTER);
CommandController partner(pros::controller_id_e_t::E_CONTROLLER_PARTNER);

pros::Distance intakeDistance(21);

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
	CommandScheduler::registerSubsystem(lift, lift->positionCommand(0.0));
	CommandScheduler::registerSubsystem(goalClamp, goalClamp->levelCommand(false));
	CommandScheduler::registerSubsystem(hook, hook->positionCommand(0.0));

	loadOneRingHigh = new Sequence({
				new ParallelRaceGroup({
					bottomIntake->movePct(1.0),
					lift->positionCommand(0.0_deg),
					topIntake->positionCommand(0.3),
					new WaitUntilCommand([&]() { return intakeDistance.get() < 100; })
				}),
				new ParallelRaceGroup({
					bottomIntake->movePct(1.0),
					lift->positionCommand(0.0_deg),
					topIntake->positionCommand(0.3),
					new WaitCommand(0.1_s)
				}),
				new ParallelRaceGroup({
					bottomIntake->movePct(1.0),
					lift->positionCommand(0.0_deg),
					new ParallelCommandGroup({topIntake->moveToPosition(1.3), new WaitCommand(1.0_s)}),
				}),
			});
	loadOneRingLow = new Sequence({
				new ParallelRaceGroup({
					bottomIntake->movePct(1.0),
					lift->positionCommand(10.0_deg),
					topIntake->positionCommand(-0.1),
					new WaitUntilCommand([&]() { return intakeDistance.get() < 100; })
				}),
				new ParallelRaceGroup({
					bottomIntake->movePct(1.0),
					lift->positionCommand(10.0_deg),
					topIntake->positionCommand(-0.1),
					new WaitCommand(0.1_s)
				}),
				new ParallelRaceGroup({
					bottomIntake->movePct(1.0),
					lift->positionCommand(10.0_deg),
					new ParallelCommandGroup({topIntake->moveToPosition(-1.1), new WaitCommand(1.0_s)}),
				}),
			});
	intakeOntoGoal = new ParallelCommandGroup({
		bottomIntake->movePct(1.0),
		lift->positionCommand(0.0_deg),
		topIntake->movePct(1.0)
	});

	primary.getTrigger(DIGITAL_L1)->toggleOnTrue(
		new RepeatCommand(
			loadOneRingHigh
		)
	);

	primary.getTrigger(DIGITAL_L2)->toggleOnTrue(
		new RepeatCommand(
			loadOneRingLow
		)
	);

	primary.getTrigger(DIGITAL_R2)->toggleOnTrue(intakeOntoGoal);
	primary.getTrigger(DIGITAL_R1)->onTrue(new Sequence({
		new ParallelRaceGroup({
			bottomIntake->movePct(0.0),
			lift->positionCommand(30_deg),
			topIntake->movePct(0.0),
			hook->positionCommand(5_deg),
			new WaitUntilCommand([&]() { return primary.get_digital(DIGITAL_Y); })
		}),
		new ParallelCommandGroup({
			bottomIntake->movePct(0.0),
			lift->positionCommand(30_deg),
			topIntake->movePct(-1.0)
		})
	}))->onFalse(new ParallelRaceGroup({
		bottomIntake->movePct(0.0),
		lift->moveToPosition(0_deg, 10_deg),
		topIntake->movePct(0.7),
		hook->positionCommand(5_deg),
	}));

	primary.getTrigger(DIGITAL_RIGHT)->toggleOnTrue(goalClamp->levelCommand(true));
}
