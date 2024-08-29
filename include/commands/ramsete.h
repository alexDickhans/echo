#pragma once

#include "command/command.h"
#include "subsystems/drivetrain.h"
#include "units/units.hpp"
#include "motionProfiling/motionProfile.h"

class Ramsete : public Command {
private:
	Drivetrain* drivetrain;

	double zeta;
	double beta;

	QTime startTime;

	MotionProfile* motionProfile;
public:
	Ramsete(Drivetrain *drivetrain, double zeta, double beta, MotionProfile *motion_profile)
		: drivetrain(drivetrain),
		  zeta(zeta),
		  beta(beta),
		  motionProfile(motion_profile) {
		startTime = 0.0;
	}

	void initialize() override {
		startTime = pros::millis() * millisecond;
	}

	void execute() override {
		auto command = motionProfile->get(pros::millis() * millisecond - startTime);

		if (command.has_value()) {

		}
	}

	bool isFinished() override {
		return motionProfile->getDuration() < pros::millis() * millisecond - startTime;
	}

	std::vector<Subsystem *> getRequirements() override {
		return {drivetrain};
	}
};