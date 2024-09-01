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
		intakeMotor.set_gearing(pros::MotorGears::green);
	}

	void periodic() override {
		// No-op
	}

	void setPosition(double position) const {
		this->intakeMotor.move_absolute(position * CONFIG::INTAKE_RATIO, 200);
	}

	void setPct(double pct) const {
		this->intakeMotor.move_voltage(pct * 12000.0);
	}

	double getPosition() {
		return this->intakeMotor.get_position() / CONFIG::INTAKE_RATIO;
	}

	double error() {
		return this->getPosition() - this->intakeMotor.get_target_position() / CONFIG::INTAKE_RATIO;
	}

	RunCommand *stopIntake() {
		return new RunCommand([&]() { this->setPct(0.0); }, {this});
	}

	FunctionalCommand *positionCommand(double position) {
		return new FunctionalCommand(
			[this, position]() mutable {
				this->setPosition(static_cast<double>(static_cast<int>(this->getPosition()) / 1) + position);
			}, [this]() {  }, [](bool _) {
			}, [this]() { return false; }, {this});
	}

	FunctionalCommand *moveToPosition(double position) {
		return new FunctionalCommand(
			[this, position]() mutable {
				this->setPosition(static_cast<double>(static_cast<int>(this->getPosition()) / 1) + position);
			}, [this]() {  }, [](bool _) {
			}, [this]() { return this->error() < 0.02; }, {this});
	}

	RunCommand *movePct(double pct) {
		return new RunCommand([this, pct]() { this->setPct(pct); }, {this});
	}

	~TopIntake() override = default;
};
