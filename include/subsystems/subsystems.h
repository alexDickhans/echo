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
#include "commands/fillIntake.h"

Drivetrain *drivetrain;
TopIntake *topIntake;
BottomIntake *bottomIntake;
LiftSubsystem *lift;
GoalClamp *goalClamp;
Hook* hook;

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

	primary.getTrigger(DIGITAL_L1)->toggleOnTrue(
		new RepeatCommand(new FillIntake(bottomIntake, topIntake, lift, &intakeDistance)));

	primary.getTrigger(DIGITAL_R2)->toggleOnTrue(new ParallelCommandGroup({
		bottomIntake->movePct(1.0),
		lift->positionCommand(0.0_deg),
		topIntake->movePct(1.0)
	}));
	primary.getTrigger(DIGITAL_R1)->onTrue(new Sequence( {
		new ParallelRaceGroup({
			bottomIntake->movePct(0.0),
			lift->positionCommand(30_deg),
			topIntake->movePct(0.0),
			hook->positionCommand(10_deg),
			new WaitUntilCommand([&] () { return primary.get_digital(DIGITAL_Y); })
		}), new ParallelCommandGroup({
			bottomIntake->movePct(0.0),
			lift->positionCommand(30_deg),
			topIntake->movePct(-1.0)
		})
	}))->onFalse(new ParallelRaceGroup({
			bottomIntake->movePct(0.0),
			lift->moveToPosition(0_deg, 10_deg),
			topIntake->movePct(0.7),
			hook->positionCommand(10_deg),
		}));

	primary.getTrigger(DIGITAL_A)->toggleOnTrue(hook->positionCommand(20.0_deg));
	primary.getTrigger(DIGITAL_RIGHT)->toggleOnTrue(goalClamp->levelCommand(true));
}
