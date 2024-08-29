#pragma once

#include "command/subsystem.h"

class Hook : public Subsystem {
private:
	pros::Motor motor;
public:
	explicit Hook(pros::Motor motor)
		: motor(std::move(motor)) {
		motor.set_gearing_all(pros::MotorGears::green);
		motor.set_brake_mode(pros::MotorBrake::hold);
		motor.set_encoder_units_all(pros::E_MOTOR_ENCODER_ROTATIONS);
	}

	void periodic() override {}

	void setPosition(const Angle angle) {
		motor.move_absolute(angle.Convert(revolution), 200);
	}

	FunctionalCommand* positionCommand(Angle angle) {
		return new FunctionalCommand([this, angle]() { this->setPosition(angle); }, []() {}, [](bool _) {}, []() {return false;}, {this});
	}

	~Hook() override = default;
};