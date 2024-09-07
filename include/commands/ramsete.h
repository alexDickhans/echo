#pragma once

#include "subsystems/drivetrain.h"
#include "command/command.h"
#include "subsystems/drivetrain.h"
#include "units/units.hpp"
#include "motionProfiling/motionProfile.h"
#include "utils/utils.h"

class Ramsete : public Command {
private:
	Drivetrain *drivetrain;

	double zeta;
	double beta;

	QTime startTime;

	MotionProfile *motionProfile;

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
		if (const auto command = motionProfile->get(pros::millis() * millisecond - startTime); command.has_value()) {
			Eigen::Vector3d currentPose = drivetrain->getPose().cast<double>();

			const Eigen::Matrix3d errorMatrix{
				{cos(currentPose.z()), sin(currentPose.z()), 0.0},
				{-sin(currentPose.z()), cos(currentPose.z()), 0.0},
				{0.0, 0.0, 1.0}
			};

			Eigen::Vector3d error = errorMatrix * (command->desiredPose - currentPose);

			error.z() = angleDifference(command->desiredPose.z(), currentPose.z()).getValue();

			auto k = 2.0 * this->zeta * sqrt(Qsq(command->desiredAngularVelocity).getValue()
			                                 + this->beta * Qsq(command->desiredVelocity).getValue());

			auto velocity_commanded = (command->desiredVelocity.getValue() * cos(error.z()) + k * error.x()) * metre /
			                          second;
			auto angular_wheel_velocity_commanded = (command->desiredAngularVelocity.getValue() + k * angleDifference(
				                                         error.z() * radian, 0.0).getValue()
			                                         + this->beta
			                                         * command->desiredVelocity.getValue()
			                                         * sinc(error.z())
			                                         * error.y())
			                                        * CONFIG::TRACK_WIDTH
			                                        / 2.0 / second;

			drivetrain->setVelocity(velocity_commanded - angular_wheel_velocity_commanded,
			                        velocity_commanded + angular_wheel_velocity_commanded);

		}
	}

	void end(bool interrupted) override {
          std::cout << "DONE" << std::endl;
	}

	bool isFinished() override {
		return motionProfile->getDuration() < pros::millis() * millisecond - startTime;
	}

	std::vector<Subsystem *> getRequirements() override {
		return {drivetrain};
	}
};
