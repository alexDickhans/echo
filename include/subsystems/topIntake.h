#pragma once
#include <config.h>

#include "command/command.h"
#include "command/runCommand.h"

class TopIntake : public Subsystem {
	pros::Motor intakeMotor;
public:
	explicit TopIntake(pros::Motor intake_motor)
		: intakeMotor(std::move(intake_motor)) {
		intakeMotor.set_encoder_units(pros::MotorEncoderUnits::rotations);
	}

	void periodic() override {
		// No-op
	}

	void setPosition(double position) const {
		this->intakeMotor.move_absolute(position * CONFIG::INTAKE_RATIO, 600);
	}

	void setPct(double pct) const {
		this->intakeMotor.move_voltage(pct * 12000.0);
	}

	RunCommand* stopIntake() {
		return new RunCommand([&]() {this->setPct(0.0);}, {this});
	}

	~TopIntake() override = default;
};
