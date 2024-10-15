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
		motor.move_absolute(angle.Convert(revolution) * 2.0, 200);
	}

	void setPct(const float pct) {
		motor.move_voltage(12000.0f * pct);
	}

	FunctionalCommand* positionCommand(Angle angle) {
		return new FunctionalCommand([this, angle]() { this->setPosition(angle); }, []() {}, [](bool _) {}, []() {return false;}, {this});
	}

	FunctionalCommand* pctCommand(const float pct) {
		return new FunctionalCommand([this, pct]() { this->setPct(pct); }, []() {}, [](bool _) {}, []() {return false;}, {this});
	}

	~Hook() override = default;
};