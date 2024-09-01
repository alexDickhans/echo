#pragma once


#include <utility>

#include "config.h"
#include "velocityProfile/trapezoidalVelocityProfile.hpp"
#include "subsystems/drivetrain.h"
#include "command/command.h"

class TankMotionProfiling : public Command {
private:

	Drivetrain* drivetrain;
	TrapezoidalVelocityProfile velocityProfile;

	QTime startTime = 0.0;

	Angle targetAngle = 0.0;

	QLength distance;
	QCurvature curvature;

	QLength startDistance;

	bool useTurnPID = true;

	PID distancePid = CONFIG::DISTANCE_PID;
	PID anglePid = CONFIG::TURN_PID;

public:
	TankMotionProfiling(Drivetrain *drivetrain, ProfileConstraints profile_constraints,
		const QLength &distance, const QCurvature &curvature = 0.0, const bool useTurnPID = true, const QVelocity initialVelocity = 0.0, const QVelocity endVelocity = 0.0, const Angle &target_angle = 0.0)
		: drivetrain(drivetrain),
		  velocityProfile(distance, profile_constraints, initialVelocity, endVelocity),
		  targetAngle(target_angle),
		  curvature(curvature) {
		this->useTurnPID = useTurnPID;
	}

	double getSpeedMultiplier() {

		if (curvature.getValue() == 0.0)
			return 1.0;

		return 1.0/(1.0 + abs(curvature.getValue() * 0.5) * CONFIG::TRACK_WIDTH.getValue());
	}

	void initialize() override {
		startTime = pros::millis() * 1_ms;

		startDistance = drivetrain->getDistance();

		QVelocity adjustedSpeed = this->getSpeedMultiplier() * this->velocityProfile.getProfileConstraints().maxVelocity.getValue();

		this->velocityProfile.setProfileConstraints({adjustedSpeed, velocityProfile.getProfileConstraints().maxAcceleration});

		this->velocityProfile.calculate();
	}

	void execute() override {
		QTime duration = (pros::millis() * 1_ms) - startTime;

		QAcceleration acceleration = velocityProfile.getAccelerationByTime(duration);
		QVelocity speed = velocityProfile.getVelocityByTime(duration);
		QLength targetDistance = velocityProfile.getDistanceByTime(duration);

		QLength currentDistance = (drivetrain->getDistance()-startDistance);

		distancePid.setTarget(targetDistance.getValue());

		const double wheelVoltage = distancePid.update(currentDistance.getValue()) + CONFIG::DRIVETRAIN_FEEDFORWARD(speed, acceleration);

		// add curvature
		const double leftCurvatureAdjustment = (2.0 + curvature.getValue() * CONFIG::TRACK_WIDTH.getValue()) / 2.0;
		const double rightCurvatureAdjustment = (2.0 - curvature.getValue() * CONFIG::TRACK_WIDTH.getValue()) / 2.0;

		double leftVoltage = leftCurvatureAdjustment * wheelVoltage;
		double rightVoltage = rightCurvatureAdjustment * wheelVoltage;

		// integrate turnPid
		if (useTurnPID) {
			Angle offset = targetDistance * curvature;

			Angle targetAngleWithOffset = targetAngle + offset;

			CONFIG::TURN_PID.setTarget(targetAngleWithOffset.getValue());

			const double turnPower = CONFIG::TURN_PID.update(drivetrain->getPose().z());

			leftVoltage -= turnPower;
			rightVoltage += turnPower;
		}

		std::cout << "Left: " << leftVoltage << ", right: " << rightVoltage << std::endl;

		// send to motors
		drivetrain->setPct(leftVoltage, rightVoltage);
	}

	void end(bool interrupted) override {

	}

	std::vector<Subsystem *> getRequirements() override {
		return {drivetrain};
	}

	bool isFinished() override {
		return (pros::millis() * 1_ms) - startTime > velocityProfile.getDuration();
	}

	~TankMotionProfiling() override = default;
};