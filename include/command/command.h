#pragma once

#include "units/units.hpp"
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

    void schedule();
    void cancel();
    [[nodiscard]] bool scheduled() const;

    Command* andThen(Command* other);
    Command* withTimeout(QTime duration);
    Command* until(const std::function<bool()>& isFinish);
    Command* with(Command* other);
    Command* race(Command* other);
    Command* repeatedly();

    virtual ~Command() = default;
};