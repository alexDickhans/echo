#pragma once

#include "config.h"

#include "command/command.h"
#include <iostream>

/**
 * Turn on point code
 */
class Rotate : public Command {
private:
    DrivetrainSubsystem *drivetrain;
    PID pid{CONFIG::TURN_PID_NO_GOAL};
    double static_voltage;
    bool finish{true};

    std::function<Angle()> updateAngle;

public:
    Rotate(DrivetrainSubsystem *drivetrain, const Angle angle, const bool flip, const double static_voltage = 0.0,
           const bool finish = true)
        : drivetrain(drivetrain),
          static_voltage(static_voltage),
          finish(finish), updateAngle([flip, angle]() { return (flip ? -1.0f : 1.0f) * angle.getValue(); }) {
    }

    Rotate(DrivetrainSubsystem *drivetrain, const std::function<Angle()> &updateAngle, const bool flip,
           const double static_voltage = 0.0,
           const bool finish = true)
        : drivetrain(drivetrain),
          static_voltage(static_voltage),
          finish(finish), updateAngle([flip, updateAngle] () { return (flip ? -1.0f : 1.0f) * updateAngle().getValue(); }) {
    }

    void initialize() override {
        this->pid = drivetrain->robotHasGoal() ? CONFIG::TURN_PID_GOAL : CONFIG::TURN_PID_NO_GOAL;
        this->pid.reset();
        this->pid.setTarget(updateAngle().getValue());
        this->pid.setTurnPid(true);
    }

    void execute() override {
        const auto output = std::ranges::clamp(pid.update(drivetrain->getAngle().getValue()), -1.0, 1.0);
        drivetrain->setPct(static_voltage - output, static_voltage + output);
    }

    bool isFinished() override {
        return finish && Qabs(angleDifference(this->drivetrain->getAngle(), pid.getTarget() * radian)) <
               CONFIG::ANGLE_FINISH_THRESHOLD;
    }

    std::vector<Subsystem *> getRequirements() override {
        return {drivetrain};
    }

    ~Rotate() override = default;
};
