#pragma once

#include "command/commandScheduler.h"
#include "lift.h"
#include "topIntake.h"
#include "goalClamp.h"
#include "bottomIntake.h"
#include "drivetrain.h"

Drivetrain* drivetrain;
TopIntake* topIntake;
BottomIntake* bottomIntake;
LiftSubsystem* lift;

pros::Controller primary(pros::controller_id_e_t::E_CONTROLLER_MASTER);
pros::Controller partner(pros::controller_id_e_t::E_CONTROLLER_PARTNER);

void subsytemInit() {
	drivetrain = new Drivetrain({}, {}, {}, {}, pros::Imu(16)); // TODO: actual ports
	topIntake = new TopIntake(pros::Motor(8));
	bottomIntake = new BottomIntake(pros::Motor(8));
	lift = new LiftSubsystem(pros::Motor(5), PID(0.0, 0.0, 0.0));

	CommandScheduler::registerSubsystem(drivetrain, drivetrain->joystick(primary));
	CommandScheduler::registerSubsystem(topIntake, topIntake->stopIntake());
	CommandScheduler::registerSubsystem(bottomIntake, topIntake->stopIntake());
	CommandScheduler::registerSubsystem(lift, lift->positionCommand(0.0));
}