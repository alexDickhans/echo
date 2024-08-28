#pragma once

#include "subsystem.h"

// enum for different cancel behaviors
enum class CommandCancelBehavior {
    CancelIncoming,
    CancelRunning,
};

// Like WPILib's Command class
class Command {
public:
    virtual void initialize() {}
    virtual void execute() {}
    virtual bool isFinished() { return false; };
    virtual void end(bool interrupted) {};
    virtual std::vector<Subsystem*> getRequirements() { return {}; };
    virtual CommandCancelBehavior getCancelBehavior() { return CommandCancelBehavior::CancelRunning; };
    virtual ~Command() = default;
};