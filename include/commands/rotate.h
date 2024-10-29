#pragma once

#include "config.h"

#include "command/command.h"
#include <iostream>

/**
 * Turn on point code
 */
class Rotate : public Command {
private:
	Drivetrain *drivetrain;
	PID pid{CONFIG::TURN_PID};
	double static_voltage;
	bool finish{true};

public:
	Rotate(Drivetrain *drivetrain, const Angle angle, const bool flip, const double static_voltage = 0.0,
           const bool finish = true)
		: drivetrain(drivetrain),
		  static_voltage(static_voltage),
		  finish(finish) {
		this->pid.setTarget((flip ? -1.0f : 1.0f) * angle.getValue());
	}

	void initialize() override {
		pid.reset();
		this->pid.setTurnPid(true);
	}

	void execute() override {
		const auto output = std::ranges::clamp(pid.update(drivetrain->getAngle().getValue()), -1.0, 1.0);
		drivetrain->setPct(static_voltage - output, static_voltage + output);
	}

	bool isFinished() override {
		return finish && Qabs(angleDifference(this->drivetrain->getAngle(), pid.getTarget() * radian)) < CONFIG::ANGLE_FINISH_THRESHOLD; // && abs(pid.getDerivitive()) < CONFIG::ANGLE_DA_FINISH_THRESHOLD;
	}

	std::vector<Subsystem *> getRequirements() override {
		return {drivetrain};
	}

	~Rotate() override = default;
};
