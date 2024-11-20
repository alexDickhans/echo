#pragma once

#include "command/command.h"
#include "config.h"
#include "feedback/pid.h"
#include "subsystems/topIntake.h"
#include "velocityProfile/trapProfile.h"

#include <iostream>

class TrapTopPosition : public Command {
private:
    TopIntake *intake;
    PID pid;
    float tolerance;

    TrapProfile::State setpoint;

    std::function<TrapProfile::State(float)> setpointCallback;

    TrapProfile profile;
public:

    TrapTopPosition(TopIntake *intake, float tolerance,
                    const std::function<TrapProfile::State(float)> &setpointCallback, const PID &pid,
                    const TrapProfile &profile);

    static TrapTopPosition *fromReversePositionCommand(TopIntake *intake, float setpoint, TrapProfile profile,
                                                       float tolerance = CONFIG::TOP_INTAKE_DEFAULT_TOLERANCE,
                                                       PID pid = CONFIG::TOP_INTAKE_PID);

    static TrapTopPosition *fromForwardPositionCommand(TopIntake *intake, float setpoint, TrapProfile profile,
                                                       float tolerance = CONFIG::TOP_INTAKE_DEFAULT_TOLERANCE,
                                                       PID pid = CONFIG::TOP_INTAKE_PID);

    static TrapTopPosition *fromClosePositionCommand(TopIntake *intake, float setpoint, TrapProfile profile,
                                                     float tolerance = CONFIG::TOP_INTAKE_DEFAULT_TOLERANCE,
                                                     PID pid = CONFIG::TOP_INTAKE_PID);

    void initialize() override;

    void execute() override;

    bool isFinished() override;

    std::vector<Subsystem *> getRequirements() override;

    ~TrapTopPosition() override;
};
