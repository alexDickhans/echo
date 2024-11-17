#pragma once

#include <utility>

#include "command/command.h"
#include "config.h"
#include "feedback/pid.h"
#include "subsystems/topIntake.h"
#include "velocityProfile/trapProfile.h"

class TopIntakePositionCommand : public Command {
private:
    TopIntake *intake;
    PID pid;
    float tolerance;
    TrapProfile::State target{0.0, 0.0};

    std::optional<std::function<float(float)>> positionCallback;

    std::optional<TrapProfile> profile;

public:
    TopIntakePositionCommand(TopIntake *intake, float tolerance, const std::function<float(float)> &position_callback,
                             const PID &pid, std::optional<TrapProfile> profile) :
        intake(intake), pid(pid), tolerance(tolerance), positionCallback(position_callback),
        profile(std::move(profile)) {
        this->pid.setTarget(position_callback(this->intake->getPosition()));
    }

    TopIntakePositionCommand(TopIntake *intake, float setpoint, float tolerance, PID pid,
                             std::optional<TrapProfile> profile) :
        intake(intake), pid(pid), tolerance(tolerance), profile(std::move(profile)) {
        this->pid.setTarget(setpoint);
    }

    static TopIntakePositionCommand *fromReversePositionCommand(TopIntake *intake, float setpoint,
                                                                float tolerance = 0.005,
                                                                std::optional<TrapProfile> profile = std::nullopt,
                                                                PID pid = CONFIG::TOP_INTAKE_PID) {
        return new TopIntakePositionCommand(
                intake, tolerance,
                [setpoint](float position) { return static_cast<float>(std::ceil(position) + setpoint); }, pid,
                profile);
    }

    static TopIntakePositionCommand *fromForwardPositionCommand(TopIntake *intake, float setpoint,
                                                                float tolerance = 0.005,
                                                                std::optional<TrapProfile> profile = std::nullopt,
                                                                PID pid = CONFIG::TOP_INTAKE_PID) {
        return new TopIntakePositionCommand(
                intake, tolerance,
                [setpoint](float position) { return static_cast<float>(std::floor(position) + setpoint); }, pid,
                profile);
    }

    static TopIntakePositionCommand *fromClosePositionCommand(TopIntake *intake, float setpoint, float tolerance = 0.005,
                                                              std::optional<TrapProfile> profile = std::nullopt,
                                                              PID pid = CONFIG::TOP_INTAKE_PID) {
        return new TopIntakePositionCommand(
                intake, tolerance,
                [setpoint](float position) { return static_cast<float>(std::round(position) + setpoint); }, pid,
                profile);
    }

    void initialize() override {
        pid.reset();

        if (positionCallback.has_value()) {
            if (profile.has_value()) {
                target = {positionCallback.value()(this->intake->getPosition()), 0.0};
            } else {
                pid.setTarget(positionCallback.value()(this->intake->getPosition()));
            }
        }
    }

    void execute() override {
        if (profile.has_value()) {
            pid.setTarget(profile->calculate(0.01, {intake->getPosition(), intake->getVelocity()}, target).position);
            intake->setPct(pid.update(intake->getPosition()));
            std::cout << pid.getTarget() << std::endl;
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
