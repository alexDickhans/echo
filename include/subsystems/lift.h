#pragma once

#include <utility>
#include "command/runCommand.h"

#include "command/subsystem.h"
#include "units/units.hpp"
#include "feedback/pid.h"
#include "config.h"

class LiftSubsystem : public Subsystem {
private:
	pros::MotorGroup motor;
	pros::Rotation rotation;

	PID pid;
public:
	explicit LiftSubsystem(const std::initializer_list<int8_t> &motors, pros::Rotation rotation, const PID& pid) : motor(motors), pid(pid), rotation(rotation) {
		motor.set_encoder_units_all(pros::MotorEncoderUnits::rotations);
	}

	void periodic() override {
		auto command = pid.update(this->getPosition().Convert(radian));
		motor.move_voltage(command * 12000.0);
		std::cout << getPosition().Convert(degree) << std::endl;
	}

	void setTarget(Angle angle) {
		pid.setTarget(angle.Convert(radian));
	}

	Angle getPosition() const {
		return angleDifference((rotation.get_angle() / 100.0) * degree, 0_deg);
	}

	RunCommand* positionCommand(Angle angle) {
		return new RunCommand([this, angle]() { this->setTarget(angle); }, {this});
	}

	FunctionalCommand* moveToPosition(Angle angle, Angle threshold) {
		return new FunctionalCommand([]() {}, [this, angle]() { this->setTarget(angle); }, [](bool _) {}, [this, threshold, angle]() { return Qabs(this->getPosition() - angle) < threshold; }, {this});
	}

	RunCommand* controller(pros::Controller* controller) {
		return new RunCommand([this, controller]() { this->setTarget(this->getPosition() + controller->get_analog(ANALOG_LEFT_Y) / 127.0 * 1.0_deg); }, {this});
	}

	~LiftSubsystem() override = default;
};