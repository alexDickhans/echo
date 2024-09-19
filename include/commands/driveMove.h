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
	TankMotionProfiling(Drivetrain *drivetrain, const ProfileConstraints &profile_constraints,
		const QLength &distance, const bool flip, const Angle &target_angle, const QCurvature &curvature = 0.0, const bool useTurnPID = true, const QVelocity initialVelocity = 0.0, const QVelocity endVelocity = 0.0)
		: drivetrain(drivetrain),
		  velocityProfile(distance, profile_constraints, initialVelocity, endVelocity),
		  targetAngle((flip ? -1.0f : 1.0f) * target_angle),
		  curvature((flip ? -1.0f : 1.0f) * curvature.getValue()) {
		this->useTurnPID = useTurnPID;
		anglePid.setTurnPid(true);
	}

	[[nodiscard]] double getSpeedMultiplier() const {

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

		anglePid.setTarget(targetAngle.getValue());

		anglePid.reset();
	}

	void execute() override {
		const QTime duration = (pros::millis() * 1_ms) - startTime;

		const QAcceleration acceleration = velocityProfile.getAccelerationByTime(duration);
		const QVelocity speed = velocityProfile.getVelocityByTime(duration);
		const QLength targetDistance = velocityProfile.getDistanceByTime(duration);

		const QLength currentDistance = (drivetrain->getDistance()-startDistance);

		distancePid.setTarget(targetDistance.getValue());

		const double wheelVoltage = distancePid.update(currentDistance.getValue()) + CONFIG::DRIVETRAIN_FEEDFORWARD(speed, acceleration);

		// add curvature
		const double leftCurvatureAdjustment = (2.0 + curvature.getValue() * CONFIG::TRACK_WIDTH.getValue()) / 2.0;
		const double rightCurvatureAdjustment = (2.0 - curvature.getValue() * CONFIG::TRACK_WIDTH.getValue()) / 2.0;

		double leftVoltage = leftCurvatureAdjustment * wheelVoltage;
		double rightVoltage = rightCurvatureAdjustment * wheelVoltage;

		// integrate turnPid
		if (useTurnPID) {
			const Angle offset = targetDistance * curvature;

			const Angle targetAngleWithOffset = targetAngle + offset;

			anglePid.setTarget(targetAngleWithOffset.getValue());

			const double turnPower = std::clamp(anglePid.update(drivetrain->getPose().z()), -1.0, 1.0);

			leftVoltage -= turnPower;
			rightVoltage += turnPower;
		}

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