#pragma once

#include "config.h"

#include "command/command.h"
#include "command/runCommand.h"
#include "pros/aivision.hpp"


enum RingColor_ { Blue = 2, Red = 1, None = 0 } typedef RingColor;

inline pros::aivision_color_s RED_COLOR_DESC(1, 243, 104, 122, 17, 0.2);
inline pros::aivision_color_s BLUE_COLOR_DESC(2, 92, 229, 247, 20, 0.2);

class TopIntake : public Subsystem {
	pros::MotorGroup intakeMotor;
	pros::AIVision vision;
	pros::Distance intakeDistance;

public:
	explicit TopIntake(const std::initializer_list<int8_t> &motors, pros::Distance intakeDistance, pros::AIVision vision)
		: intakeMotor(motors), vision(vision), intakeDistance(std::move(intakeDistance)) {
		intakeMotor.set_encoder_units_all(pros::MotorEncoderUnits::rotations);
		intakeMotor.set_gearing(pros::MotorGears::green);
		vision.set_color(RED_COLOR_DESC);
		vision.set_color(BLUE_COLOR_DESC);
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
		return (this->intakeMotor.get_position(0) + this->intakeMotor.get_position(1)) / (2 * CONFIG::INTAKE_RATIO);
	}

	double getVelocity() {
		return (this->intakeMotor.get_actual_velocity(0) + this->intakeMotor.get_actual_velocity(1)) / (2 * CONFIG::INTAKE_RATIO);
	}

	bool ringPresent() {
		return intakeDistance.get() < 40;
	}

	RingColor getRingColor() {
		if (vision.get_object_count() > 0)
			return static_cast<RingColor>(vision.get_object(0).id);
		return RingColor::None;
	}

	RunCommand *pctCommand(double pct) {
		return new RunCommand([this, pct]() { this->setPct(pct); }, {this});
	}

	RunCommand* controllerCommand(pros::Controller* controller) {
		return new RunCommand([this, controller]() { this->setPct(controller->get_analog(ANALOG_RIGHT_Y) / 127.0); }, {this});
	}

	~TopIntake() override = default;
};
