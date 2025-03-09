#pragma once

#pragma once

#include "command/command.h"
#include "subsystems/lift.h"
#include "velocityProfile/trapProfile.h"

class TrapLiftPosition : public Command {
private:
    LiftSubsystem *lift;
    Angle tolerance;

    TrapProfile::State setpoint;

    TrapProfile profile;

public:
    TrapLiftPosition(LiftSubsystem *lift, const Angle tolerance, const TrapProfile &profile,
                     const TrapProfile::State &setpoint) :
        lift(lift), tolerance(tolerance), setpoint(setpoint), profile(profile) {}

    void initialize() override {}

    void execute() override {
        auto angle =
                profile.calculate(100_ms,
                                  {lift->getPosition().Convert(degree), lift->getVelocity().Convert(degree / second)},
                                  setpoint);
        lift->setTarget(angle.position * 1_deg);
    }

    bool isFinished() override {
        if (tolerance.getValue() != 0.0) {
            return Qabs(lift->getPosition() - setpoint.position * 1_deg) < tolerance;
        }
        return false;
    }

    std::vector<Subsystem *> getRequirements() override { return {lift}; }

    ~TrapLiftPosition() override = default;
};
