#pragma once

#include <utility>

#include "command/command.h"
#include "config.h"
#include "subsystems/drivetrain.h"
#include "velocityProfile/trapezoidalVelocityProfile.hpp"

/**
* 1D Motion Profiled movements for moving straight lines and simple arcs
*/
class TankMotionProfiling : public Command {
private:
    DrivetrainSubsystem *drivetrain;
    TrapezoidalVelocityProfile velocityProfile;

    QTime startTime = 0.0;

    Angle targetAngle = 0.0;

    QLength distance;
    QCurvature curvature;

    QLength startDistance;

    bool useTurnPID = true;

    PID distancePid = CONFIG::DISTANCE_PID;
    PID anglePid = CONFIG::TURN_PID_NO_GOAL;

    std::optional<std::function<Angle(QTime)> > angleFunction;

public:
    TankMotionProfiling(DrivetrainSubsystem *drivetrain, const ProfileConstraints &profile_constraints,
                        const QLength &distance,
                        const bool flip, const Angle &target_angle, const QCurvature &curvature = 0.0,
                        const bool useTurnPID = true, const QVelocity initialVelocity = 0.0,
                        const QVelocity endVelocity = 0.0,
                        std::optional<std::function<Angle(QTime)> > angleFunction =
                                std::nullopt) : drivetrain(drivetrain),
                                                velocityProfile(distance, profile_constraints, initialVelocity,
                                                                endVelocity),
                                                targetAngle((flip ? -1.0f : 1.0f) * target_angle),
                                                curvature((flip ? -1.0f : 1.0f) * curvature.getValue()),
                                                angleFunction(std::move(angleFunction)) {
        this->useTurnPID = useTurnPID;
        anglePid.setTurnPid(true);
    }

    [[nodiscard]] double getSpeedMultiplier() const {
        if (curvature.getValue() == 0.0)
            return 1.0;

        return 1.0 / (1.0 + abs(curvature.getValue() * 0.5) * CONFIG::TRACK_WIDTH.getValue());
    }

    void initialize() override {
        startTime = pros::millis() * 1_ms;

        startDistance = drivetrain->getDistance();

        QVelocity adjustedSpeed =
                this->getSpeedMultiplier() * this->velocityProfile.getProfileConstraints().maxVelocity.getValue();

        this->velocityProfile.setProfileConstraints(
            {adjustedSpeed, velocityProfile.getProfileConstraints().maxAcceleration});

        this->velocityProfile.calculate();

        anglePid = drivetrain->robotHasGoal() ? CONFIG::TURN_PID_GOAL : CONFIG::TURN_PID_NO_GOAL;

        anglePid.setTarget(targetAngle.getValue());
        anglePid.setTurnPid(true);

        anglePid.reset();
    }

    void execute() override {
        const QTime duration = (pros::millis() * 1_ms) - startTime;

        const QAcceleration acceleration = velocityProfile.getAccelerationByTime(duration);
        const QVelocity speed = velocityProfile.getVelocityByTime(duration);
        const QLength targetDistance = velocityProfile.getDistanceByTime(duration);

        const QLength currentDistance = (drivetrain->getDistance() - startDistance);

        distancePid.setTarget(targetDistance.getValue());

        const auto [linearFF, angularFF] = CONFIG::DRIVETRAIN_FEEDFORWARD(speed, acceleration, speed * curvature,
                                                               acceleration * curvature);

        const double wheelVoltage = distancePid.update(currentDistance.getValue()) + linearFF;

        // add curvature
        const double leftCurvatureAdjustment = (2.0 + curvature.getValue() * CONFIG::TRACK_WIDTH.getValue()) / 2.0;
        const double rightCurvatureAdjustment = (2.0 - curvature.getValue() * CONFIG::TRACK_WIDTH.getValue()) / 2.0;

        double leftVoltage = leftCurvatureAdjustment * wheelVoltage;
        double rightVoltage = rightCurvatureAdjustment * wheelVoltage;

        // integrate turnPid
        if (useTurnPID) {
            const Angle offset = targetDistance * curvature;

            Angle targetAngleWithOffset = targetAngle + offset;

            if (angleFunction.has_value()) {
                targetAngleWithOffset += angleFunction.value()(duration);
            }

            anglePid.setTarget(targetAngleWithOffset.getValue());

            const double turnPower = std::clamp(anglePid.update(drivetrain->getPose().z()), -1.0, 1.0);

            leftVoltage -= turnPower + angularFF;
            rightVoltage += turnPower + angularFF;
        }

        // send to motors
        drivetrain->setPct(leftVoltage, rightVoltage);
    }

    void end(bool interrupted) override {
    }

    std::vector<Subsystem *> getRequirements() override { return {drivetrain}; }

    bool isFinished() override { return (pros::millis() * 1_ms) - startTime > velocityProfile.getDuration(); }

    ~TankMotionProfiling() override = default;
};
