#include "config.h"
#include "command/command.h"
#include "feedback/pid.h"
#include "subsystems/topIntake.h"

class TopIntakePositionCommand : public Command {
private:
    TopIntake *intake;
    PID pid;
    float tolerance;

public:
    TopIntakePositionCommand(TopIntake *intake, float setpoint, float tolerance, PID pid = CONFIG::TOP_INTAKE_PID) : intake(intake), pid(pid), tolerance(tolerance) {
        this->pid.setTarget(setpoint);
    }

    static TopIntakePositionCommand* fromReversePositionCommand(TopIntake *intake, float setpoint, float tolerance = 0.1, PID pid = CONFIG::TOP_INTAKE_PID) {
        return new TopIntakePositionCommand(intake, static_cast<float>(std::ceil(intake->getPosition()) + setpoint), tolerance, pid);
    }

    static TopIntakePositionCommand* fromForwardPositionCommand(TopIntake *intake, float setpoint, float tolerance = 0.1, PID pid = CONFIG::TOP_INTAKE_PID) {
        return new TopIntakePositionCommand(intake, static_cast<float>(std::floor(intake->getPosition()) + setpoint), tolerance, pid);
    }

    static TopIntakePositionCommand* fromClosePositionCommand(TopIntake *intake, float setpoint, float tolerance = 0.1, PID pid = CONFIG::TOP_INTAKE_PID) {
        return new TopIntakePositionCommand(intake, static_cast<float>(std::round(intake->getPosition()) + setpoint), tolerance, pid);
    }

    void initialize() override {
        pid.reset();
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
