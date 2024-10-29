#pragma once

#include "config.h"

#include "command/command.h"
#include "command/runCommand.h"
#include "vex/v5_vcs.h"


enum RingColor_ { Blue = 2, Red = 1, None = 0 } typedef RingColor;

inline vex::aivision::colordesc RED_COLOR_DESC(1, 243, 104, 122, 17, 0.2);
inline vex::aivision::colordesc BLUE_COLOR_DESC(2, 92, 229, 247, 20, 0.2);

class TopIntake : public Subsystem {
	pros::Motor intakeMotor;
	vex::aivision vision;
	pros::Distance intakeDistance;

public:
	explicit TopIntake(pros::Motor intake_motor, pros::Distance intakeDistance)
		: intakeMotor(std::move(intake_motor)), vision(12, RED_COLOR_DESC, BLUE_COLOR_DESC), intakeDistance(std::move(intakeDistance)) {
		intakeMotor.set_encoder_units(pros::MotorEncoderUnits::rotations);
		intakeMotor.set_gearing(pros::MotorGears::green);
		vision.colorDetection(true, true);
		vision.startAwb();
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

	bool ringPresent() {
		return intakeDistance.get() < 40;
	}

	RunCommand *stopIntake() {
		return new RunCommand([&]() { this->setPct(0.0); }, {this});
	}

	FunctionalCommand *positionCommandRwd(double position) {
		return new FunctionalCommand(
			[this, position]() mutable {
				this->setPosition(std::ceil(this->getPosition()) + position);
			}, [this]() {  }, [](bool _) {
			}, [this]() { return false; }, {this});
	}

	FunctionalCommand *moveToPositionRwd(double position) {
		return new FunctionalCommand(
			[this, position]() mutable {
				this->setPosition(std::ceil(this->getPosition()) + position);
			}, [this]() {  }, [](bool _) {
			}, [this]() { return this->error() < 0.02; }, {this});
	}

	FunctionalCommand *positionCommandFwd(double position) {
		return new FunctionalCommand(
			[this, position]() mutable {
				this->setPosition(std::floor(this->getPosition()) + position);
			}, [this]() {  }, [](bool _) {
			}, [this]() { return false; }, {this});
	}

	FunctionalCommand *positionCommandClose(double position) {
		return new FunctionalCommand(
			[this, position]() mutable {
				this->setPosition(std::round(this->getPosition()) + position);
			}, [this]() {  }, [](bool _) {
			}, [this]() { return false; }, {this});
	}

	FunctionalCommand *moveToPositionFwd(double position) {
		return new FunctionalCommand(
			[this, position]() mutable {
				this->setPosition(std::floor(this->getPosition()) + position);
			}, [this]() {  }, [](bool _) {
			}, [this]() { return this->error() < 0.02; }, {this});
	}

	RunCommand *movePct(double pct) {
		return new RunCommand([this, pct]() { this->setPct(pct); }, {this});
	}

	RunCommand* controller(pros::Controller* controller) {
		return new RunCommand([this, controller]() { this->setPct(controller->get_analog(ANALOG_RIGHT_Y) / 127.0); }, {this});
	}

	RingColor getRingColor() {
		vision.takeSnapshot(vex::aivision::ALL_COLORS);

		return static_cast<RingColor>(vision.largestObject.id);
	}

	~TopIntake() override = default;
};
