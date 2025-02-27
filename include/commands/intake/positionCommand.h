#pragma once

#include "command/command.h"
#include "config.h"
#include "feedback/pid.h"
#include "subsystems/topIntake.h"
#include "velocityProfile/trapProfile.h"

class TopIntakePositionCommand : public Command {
private:
    TopIntakeSubsystem *intake;
    PID pid;
    float tolerance;

    std::function<float(float)> positionCallback;

public:
    TopIntakePositionCommand(TopIntakeSubsystem *intake, float tolerance, const std::function<float(float)> &position_callback,
                             const PID &pid) :
        intake(intake), pid(pid), tolerance(tolerance), positionCallback(position_callback) {
        this->pid.setTarget(position_callback(this->intake->getPosition()));
    }

    static TopIntakePositionCommand *
    fromForwardPositionCommand(TopIntakeSubsystem *intake, float setpoint, float tolerance = CONFIG::TOP_INTAKE_DEFAULT_TOLERANCE,
                               PID pid = CONFIG::TOP_INTAKE_PID,
                               std::optional<std::pair<TrapProfile, TrapProfile::State>> profile = std::nullopt) {
        return new TopIntakePositionCommand(
                intake, tolerance,
                [setpoint](float position) { return static_cast<float>(std::floor(position) + setpoint); }, pid);
    }

    static TopIntakePositionCommand *
    fromClosePositionCommand(TopIntakeSubsystem *intake, float setpoint, float tolerance = CONFIG::TOP_INTAKE_DEFAULT_TOLERANCE,
                             PID pid = CONFIG::TOP_INTAKE_PID,
                             std::optional<std::pair<TrapProfile, TrapProfile::State>> profile = std::nullopt) {
        return new TopIntakePositionCommand(
                intake, tolerance,
                [setpoint](float position) { return static_cast<float>(std::round(position) + setpoint); }, pid);
    }

    void initialize() override {
        pid.reset();
        pid.setTarget(positionCallback(this->intake->getPosition()));
    }

    void execute() override {
        intake->setPct(pid.update(intake->getPosition()));
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
