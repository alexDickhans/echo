#include "trapTopPosition.h"

TrapTopPosition::TrapTopPosition(TopIntake *intake, float tolerance,
                                 const std::function<TrapProfile::State(float)> &setpointCallback, const PID &pid,
                                 const TrapProfile &profile) :
    intake(intake), pid(pid), tolerance(tolerance), setpoint(0.0, 0.0), setpointCallback(setpointCallback),
    profile(profile) {}
TrapTopPosition *TrapTopPosition::fromReversePositionCommand(TopIntake *intake, float setpoint, TrapProfile profile,
                                                             float tolerance, PID pid) {
    return new TrapTopPosition(
            intake, tolerance,
            [setpoint](float position) -> TrapProfile::State {
                return {(std::ceil(position) + setpoint), 0.0};
            },
            pid, profile);
}
TrapTopPosition *TrapTopPosition::fromForwardPositionCommand(TopIntake *intake, float setpoint, TrapProfile profile,
                                                             float tolerance, PID pid) {
    return new TrapTopPosition(
            intake, tolerance,
            [setpoint](float position) -> TrapProfile::State {
                return {(std::floor(position) + setpoint), 0.0};
            },
            pid, profile);
}
TrapTopPosition *TrapTopPosition::fromClosePositionCommand(TopIntake *intake, float setpoint, TrapProfile profile,
                                                           float tolerance, PID pid) {
    return new TrapTopPosition(
            intake, tolerance,
            [setpoint](float position) -> TrapProfile::State {
                return {(std::round(position) + setpoint), 0.0};
            },
            pid, profile);
}
void TrapTopPosition::initialize() {
    pid.reset();

    setpoint = setpointCallback(this->intake->getPosition());
}
void TrapTopPosition::execute() {
    pid.setTarget(profile.calculate(10_ms, {intake->getPosition(), intake->getVelocity()}, setpoint).position);
    intake->setPct(pid.update(intake->getPosition()));

    std::cout << pid.getTarget() << std::endl;
}
bool TrapTopPosition::isFinished() {
    if (tolerance != 0.0) {
        return fabs(pid.getError()) < tolerance;
    }
    return false;
}
std::vector<Subsystem *> TrapTopPosition::getRequirements() { return {intake}; }
TrapTopPosition::~TrapTopPosition() {}
