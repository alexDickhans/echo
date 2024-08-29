#pragma once
#include "config.h"

#include "command/command.h"
#include "command/runCommand.h"

class TopIntake : public Subsystem {
	pros::Motor intakeMotor;
public:
	explicit TopIntake(pros::Motor intake_motor)
		: intakeMotor(std::move(intake_motor)) {
		intakeMotor.set_encoder_units(pros::MotorEncoderUnits::rotations);
		intakeMotor.set_gearing(pros::MotorGears::blue);
	}

	void periodic() override {
		// No-op
		std::cout << this->getPosition() << std::endl;
	}

	void setPosition(double position) const {
		this->intakeMotor.move_absolute(position * CONFIG::INTAKE_RATIO, 600);
	}

	void setPct(double pct) const {
		this->intakeMotor.move_voltage(pct * 12000.0);
	}

	double getPosition() {
		return this->intakeMotor.get_position() / CONFIG::INTAKE_RATIO;
	}

	RunCommand* stopIntake() {
		return new RunCommand([&]() {this->setPct(0.0);}, {this});
	}

	RunCommand* positionCommand(double position) {
		double position2 = position;
		return new RunCommand([this, position2]() {this->setPosition(position2);}, {this});
	}

	FunctionalCommand* moveToPosition(double position) {
		double position2 = fmod(getPosition(), 1.0) + position;
		return new FunctionalCommand([]() {}, [this, position2]() {this->setPosition(position2);}, [](bool _) {}, [this, position2]() { return abs(this->getPosition() - position2) < 0.1; }, {this});
	}

	RunCommand* movePct(double pct) {
		return new RunCommand([this, pct]() {this->setPct(pct);}, {this});
	}

	~TopIntake() override = default;
};
