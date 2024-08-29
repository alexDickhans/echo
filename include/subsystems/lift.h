#pragma once

#include <utility>
#include <command/runCommand.h>

#include "command/subsystem.h"
#include "units/units.hpp"
#include "feedback/pid.h"

class LiftSubsystem : public Subsystem {
private:
	pros::Motor motor;

	PID pid;
public:
	explicit LiftSubsystem(pros::Motor motor, const PID& pid) : motor(std::move(motor)), pid(pid) {
		motor.set_encoder_units(pros::MotorEncoderUnits::rotations);
	}

	void periodic() override {
		motor.move_voltage(pid.update((motor.get_position() * revolution).Convert(radian)));
	}

	void setTarget(Angle angle) {
		pid.setTarget(angle.Convert(radian));
	}

	RunCommand* positionCommand(Angle angle) {
		return new RunCommand([this, angle]() { this->setTarget(angle); }, {this});
	}

	~LiftSubsystem() override;
};