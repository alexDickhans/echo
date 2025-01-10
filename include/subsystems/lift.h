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

	PID pid;
public:
	explicit LiftSubsystem(const std::initializer_list<int8_t> &motors, const PID& pid) : motor(motors), pid(pid) {
		motor.set_encoder_units_all(pros::MotorEncoderUnits::rotations);
		motor.tare_position_all();
	}

	void periodic() override {
		auto command = pid.update(this->getPosition().Convert(radian));
		motor.move_voltage(command * 12000.0);
	}

	void setTarget(Angle angle) {
		pid.setTarget(angle.Convert(radian));
	}

	Angle getPosition() const {
		return angleDifference((motor.get_position(1)) / CONFIG::LIFT_RATIO * revolution, 0_deg);
	}

	RunCommand* positionCommand(Angle angle) {
		return new RunCommand([this, angle]() { this->setTarget(angle); }, {this});
	}

	FunctionalCommand* moveToPosition(Angle angle, Angle threshold = 2_deg) {
		return new FunctionalCommand([]() {}, [this, angle]() { this->setTarget(angle); }, [](bool _) {}, [this, threshold, angle]() { return Qabs(this->getPosition() - angle) < threshold; }, {this});
	}

	RunCommand* controller(pros::Controller& controller, pros::controller_analog_e_t channel) {
		Angle targetPosition = 0.0;
		return new RunCommand([this, controller, targetPosition, channel]() mutable {
                    targetPosition = targetPosition + controller.get_analog(channel) / 127.0 * 1.0_deg; targetPosition = std::clamp(targetPosition, 0_deg, 130_deg); this->setTarget(targetPosition); }, {this});
	}

	~LiftSubsystem() override = default;
};