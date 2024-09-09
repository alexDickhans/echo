#pragma once

#include "command/subsystem.h"

class GoalClamp : public Subsystem {
	pros::adi::DigitalOut solenoid;
public:
	explicit GoalClamp(pros::adi::DigitalOut solenoid)
		: solenoid(std::move(solenoid)) {
	}

	void periodic() override {
		// no - op
	}

	void setLevel(const bool value) {
		solenoid.set_value(value);
	}

	RunCommand* levelCommand(bool value) {
		return new RunCommand([this, value]() { this->setLevel(value); }, {this});
	}

	~GoalClamp() override = default;
};