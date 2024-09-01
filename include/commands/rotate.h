#pragma once

#include "config.h"

#include "command/command.h"
#include <iostream>

class Rotate : public Command {
private:
	Drivetrain *drivetrain;
	PID pid;
	double static_voltage;

public:
	Rotate(Drivetrain *drivetrain, const PID& pid, const Angle angle, const double static_voltage = 0.0)
		: drivetrain(drivetrain),
		  pid(pid),
		  static_voltage(static_voltage) {
		this->pid.setTarget(angle.getValue());
		this->pid.setTurnPid(true);
	}

	void initialize() override {
		pid.reset();
	}

	void execute() override {
		const auto output = std::ranges::clamp(pid.update(drivetrain->getAngle().getValue()), -1.0, 1.0);
		drivetrain->setPct(static_voltage - output, static_voltage + output);
	}

	bool isFinished() override {
		return Qabs(angleDifference(this->drivetrain->getAngle(), pid.getTarget() * radian)) < CONFIG::ANGLE_FINISH_THRESHOLD && abs(pid.getDerivitive()) < CONFIG::ANGLE_DA_FINISH_THRESHOLD;
	}

	std::vector<Subsystem *> getRequirements() override {
		return {drivetrain};
	}

	~Rotate() override = default;
};
