#pragma once

#include "config.h"

#include "command/command.h"
#include <iostream>

/**
* This code is designed to use the AI vision sensor to track and grab the goal
*/
class DriveToGoal : public Command {
private:
	Drivetrain *drivetrain;
	PID pid;
	double static_voltage;

public:
	DriveToGoal(Drivetrain *drivetrain, const PID& pid, const double static_voltage = 0.0)
		: drivetrain(drivetrain),
		  pid(pid),
		  static_voltage(static_voltage) {
		this->pid.setTurnPid(true);
	}

	void initialize() override {
		pid.reset();
	}

	void execute() override {
		Angle angle = 0.0;

		if (const auto goalAngle = drivetrain->getGoalAngle(); goalAngle.has_value()) {
			angle = goalAngle.value();
		}

		const auto output = std::ranges::clamp(pid.update(-angle.getValue()), -1.0, 1.0);
		drivetrain->setPct(static_voltage * cos(angle) - output, static_voltage * cos(angle) + output);
	}

	bool isFinished() override {
		return false;
	}

	std::vector<Subsystem *> getRequirements() override {
		return {drivetrain};
	}

	~DriveToGoal() override = default;
};
