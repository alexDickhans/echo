#pragma once

#include "subsystem.h"
#include "units/units.hpp"
#include <vector>
#include <functional>

/**
 * @brief Enum for different cancel behaviors for Commands
 *
 * Cancel Incoming causes newly scheduled commands to fail, keeping
 * the current command running until it ends uninterrupted. Cancel
 * Running caused the currently running command to yield to the newly
 * scheduled command and end interrupted.
 *
 */
enum class CommandCancelBehavior {
    /**
     * @brief Causes the newly scheduled command(s) to fail upon
     * attempts to schedule them while a command with CancelRunning is
     * reserving the subsystem
     */
    CancelIncoming,
    /**
     * @brief Causes the currently running command to yield to the
     * newly scheduled command upon scheduling.
     */
    CancelRunning,
};

/**
 * @brief Abstract Command class for commands
 *
 * @details Commands are the base of the command based structure, they
 * compartmentalize robot behaviors and provide the structure
 * necessary to ensure that multiple commands are attempting to access
 * the same hardware at the same time. Generally in projects you can
 * use the classes such as \refitem FunctionalCommand, or \refitem
 * RunCommand for simple Command behaviors, and you can override this
 * class to create more complex behaviors
 *
 */
class Command {
public:
    /** @brief Called before every time the command is used. Users can
     *override this to create starting behaviors for custom commands
     *
     *  ```C
     *	void initialize() override {
     *		// Initialization tasks here, for example setting motor
     *position PID commands motor.move_absolute(5, 200);
     *	}
     *  ```
     */
    virtual void initialize() {}

    /**
     * @brief Execute is run every frame in the command
     *scheduler(10ms/100Hz or user selected) This can be useful for
     *updating feedback controllers such as PID or Pure Pursuit.
     *
     * ```C
     * // Example override case for PID control
     * void execute() override {
     *		motor.move_voltage(pid.update(motor.get_position()));
     * }
     * ```
     */
    virtual void execute() {}

    /**
     * @brief Command can override this function to have custom ending
     * behaviors
     *
     * @return true if the Command is finished, false if it is still
     * running
     */
    virtual bool isFinished() { return false; };

    /**
     * @brief Stops the currently running command, this can be used to
     * clean up resources or finish up certain behaviors
     *
     * @param interrupted true if the command was stopped before
     * isFinished returned true, false otherwise
     */
    virtual void end(bool interrupted){};

    /**
     * @brief This function returns the necessary subsystems needed to
     * do run command. The scheduler will work to free these
     * requirements before running the behavior to ensure no tasks
     * overlap.
     *
     * @warning You must ensure all subsystems neccesary are captured
     * by this function or else multiple commands will overlap and run
     * at the same time
     *
     * @return A vector of all the subsystem requirements for this
     * class
     */
    virtual std::vector<Subsystem *> getRequirements() { return {}; };

    /**
     * @brief Returns the cancel behavior for this class, defaults to
     * CommandCancelBehavior::CancelRunning
     *
     * @return Desired \refitem CommandCancelBehavior for the class
     */
    virtual CommandCancelBehavior getCancelBehavior() {
        return CommandCancelBehavior::CancelRunning;
    };

    /**
     * @brief Schedule this command with the \refitem CommandScheduler
     *
     * ```C
     * // Declare command(example)
     * Command* command;
     *
     * // Use shorthand to schedule the command
     * command->schedule();
     *
     * // command is now scheduled
     * ```
     */
    void schedule();

    /**
     * @brief Cancel this command if it is currently scheduled in the
     * \refitem CommandScheduler
     *
     * ```C
     * // Declare command(example)
     * Command* command;
     *
     * // Use shorthand to schedule the command
     * command->schedule();
     *
     * // command is now scheduled
     *
     * // Cancel the command
     * command->cancel();
     *
     * // command is no longer running
     * ```
     */
    void cancel();

    /**
     * @brief See if the command is currently scheduled in the
     * \refitem CommandScheduler
     *
     * ```C
     * // Declare command(example)
     * Command* command;
     *
     * // Use shorthand to schedule the command
     * command->schedule();
     *
     * // command is now scheduled
     * assert(command->scheduled() == true);
     *
     * // Cancel the command
     * command->cancel();
     *
     * // command is no longer running
     * assert(command->scheduled() == false);
     * ```
     */
    [[nodiscard]] bool scheduled() const;

    /**
     * @brief Create a \refitem Sequence with 2 commands
     *
     * @param other The command to run after the current command
     * @return A \refitem Sequence with this running first and other
     * running after
     */
    Command *andThen(Command *other);

    /**
     * @brief Make a timeout on this command
     *
     * @param duration The maximum running duration of the Command
     * @return \refitem ParallelRaceGroup with this and \refitem
     * WaitCommand of the desired duration
     */
    Command *withTimeout(QTime duration);

    /**
     * @brief Run the command until a condition is met
     *
     * @param isFinish When this condition returns true the command
     * will stop
     * @return \refitem ParallelRaceGroup with this and \refitem
     * WaitUntilCommand with the desired isFinish
     */
    Command *until(const std::function<bool()> &isFinish);

    /**
     * @brief Create a \refitem ParallelCommandGroup with this and
     * other
     *
     * @param other Other command for the \refitem
     * ParallelCommandGroup
     * @return \refitem ParallelCommandGroup with this and other
     */
    Command *with(Command *other);

    /**
     * @brief Create a \refitem ParallelRaceGroup with this and other
     *
     * @param other Other command for the \refitem ParallelRaceGroup
     * @return \refitem ParallelRaceGroup with this and other
     */
    Command *race(Command *other);

    /**
     * @brief Create a \refitem RepeatCommand with this
     *
     * @return \refitem RepeatCommand with this
     */
    Command *repeatedly();

    /**
     * @brief Create a \refitem ProxyCommand with this
     *
     * @warning Only use \refitem ProxyCommand where ABSOLUTELY
     * necessary, it can have unintended side effects. The side
     * effects are listed in the class documentation
     * @return \refitem ProxyCommand with this
     */
    Command *asProxy();

    virtual ~Command() = default;
};
