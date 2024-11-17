#pragma once

#include "config.h"
#include "command/command.h"
#include "feedback/pid.h"
#include "subsystems/topIntake.h"
#include "velocityProfile/trapProfile.h"

class TopIntakePositionCommand : public Command {
private:
    TopIntake *intake;
    PID pid;
    float tolerance;

    std::optional<std::function<float(float)>> positionCallback;

    std::optional<std::pair<TrapProfile, TrapProfile::State>> profile;
public:

    TopIntakePositionCommand(TopIntake *intake, float tolerance,
                             const std::function<float(float)> &position_callback, const PID &pid) :
        intake(intake), pid(pid), tolerance(tolerance), positionCallback(position_callback) {
        this->pid.setTarget(position_callback(this->intake->getPosition()));
    }

    TopIntakePositionCommand(TopIntake *intake, float setpoint, float tolerance, PID pid = CONFIG::TOP_INTAKE_PID) : intake(intake), pid(pid), tolerance(tolerance) {
        this->pid.setTarget(setpoint);
    }

    static TopIntakePositionCommand* fromReversePositionCommand(TopIntake *intake, float setpoint, float tolerance = 0.01, PID pid = CONFIG::TOP_INTAKE_PID, std::optional<std::pair<TrapProfile, TrapProfile::State>> profile = std::nullopt) {
        return new TopIntakePositionCommand(intake, tolerance, [setpoint] (float position) { return static_cast<float>(std::ceil(position) + setpoint); }, pid);
    }

    static TopIntakePositionCommand* fromForwardPositionCommand(TopIntake *intake, float setpoint, float tolerance = 0.01, PID pid = CONFIG::TOP_INTAKE_PID, std::optional<std::pair<TrapProfile, TrapProfile::State>> profile = std::nullopt) {
        return new TopIntakePositionCommand(intake, tolerance, [setpoint] (float position) { return static_cast<float>(std::floor(position) + setpoint); }, pid);
    }

    static TopIntakePositionCommand* fromClosePositionCommand(TopIntake *intake, float setpoint, float tolerance = 0.01, PID pid = CONFIG::TOP_INTAKE_PID, std::optional<std::pair<TrapProfile, TrapProfile::State>> profile = std::nullopt) {
        return new TopIntakePositionCommand(intake, tolerance, [setpoint] (float position) { return static_cast<float>(std::round(position) + setpoint); }, pid);
    }

    void initialize() override {
        pid.reset();

        if (positionCallback.has_value()) {
            pid.setTarget(positionCallback.value()(this->intake->getPosition()));
        }
    }

    void execute() override {
        if (profile.has_value()) {
            pid.setTarget(profile->first.calculate(0.01, {intake->getPosition(), intake->getVelocity()}, profile->second).position);
            intake->setPct(pid.update(intake->getPosition()));
        } else {
            intake->setPct(pid.update(intake->getPosition()));
        }
    }

    bool isFinished() override {
        if (tolerance != 0.0) {
            return fabs(pid.getError()) < tolerance;
        }
        return false;
    }

    std::vector<Subsystem *> getRequirements() override { return {intake}; }

    ~TopIntakePositionCommand() override = default;
};
