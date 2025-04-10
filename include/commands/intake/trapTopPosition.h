#pragma once

#include "config.h"
#include "command/command.h"
#include "feedback/pid.h"
#include "subsystems/topIntake.h"
#include "velocityProfile/trapProfile.h"

class TrapTopPosition : public Command {
private:
    TopIntakeSubsystem *intake;
    PID pid;
    float tolerance;

    TrapProfile::State setpoint;

    std::function<TrapProfile::State(float)> setpointCallback;

    TrapProfile profile;
public:

    TrapTopPosition(TopIntakeSubsystem *intake, float tolerance, const std::function<TrapProfile::State(float)> &setpointCallback,
                    const PID &pid, const TrapProfile &profile) :
        intake(intake), pid(pid), tolerance(tolerance), setpoint(0.0, 0.0), setpointCallback(setpointCallback),
        profile(profile) {
    }

    static TrapTopPosition* fromReversePositionCommand(TopIntakeSubsystem *intake, float setpoint, TrapProfile profile, float tolerance = CONFIG::TOP_INTAKE_DEFAULT_TOLERANCE, PID pid = CONFIG::TOP_INTAKE_PID) {
        return new TrapTopPosition(intake, tolerance, [setpoint] (float position) -> TrapProfile::State { return {(std::ceil(position) + setpoint), 0.0}; }, pid, profile);
    }

    static TrapTopPosition* fromForwardPositionCommand(TopIntakeSubsystem *intake, float setpoint, TrapProfile profile, float tolerance = CONFIG::TOP_INTAKE_DEFAULT_TOLERANCE, PID pid = CONFIG::TOP_INTAKE_PID) {
        return new TrapTopPosition(intake, tolerance, [setpoint] (float position) -> TrapProfile::State { return {(std::floor(position) + setpoint), 0.0}; }, pid, profile);
    }

    static TrapTopPosition* fromClosePositionCommand(TopIntakeSubsystem *intake, float setpoint, TrapProfile profile, float tolerance = CONFIG::TOP_INTAKE_DEFAULT_TOLERANCE, PID pid = CONFIG::TOP_INTAKE_PID) {
        return new TrapTopPosition(intake, tolerance, [setpoint] (float position) -> TrapProfile::State { return {(std::round(position) + setpoint), 0.0}; }, pid, profile);
    }

    void initialize() override {
        pid.reset();

        setpoint = setpointCallback(this->intake->getPosition());
    }

    void execute() override {
        pid.setTarget(profile.calculate(10_ms, {intake->getPosition(), intake->getVelocity()}, setpoint).position);
        intake->setPct(pid.update(intake->getPosition()));
    }

    bool isFinished() override {
        if (tolerance != 0.0) {
            return fabs(pid.getError()) < tolerance;
        }
        return false;
    }

    std::vector<Subsystem *> getRequirements() override { return {intake}; }

    ~TrapTopPosition() override = default;
};
