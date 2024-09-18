#pragma once

#include "command.h"

/**
 * @brief Schedules a \refitem Command as a "proxy" while tracking the progress for \refitem Sequence
 *
 * @warning Only use when you need to free a \refitem Subsystem before or after it is used in a \refitem Sequence. This command
 * should only be used when you really KNOW there is no other way to complete the action.
 */
class ProxyCommand : public Command {
private:
	Command *command;
	std::function<Command *()> supplier;

public:
	/**
	 * @brief Create a ProxyCommand with a \refitem Command pointer supplier
	 * @param supplier Supplier to get a \refitem Command pointer on initialization
	 */
	explicit ProxyCommand(const std::function<Command *()> &supplier) : supplier(supplier) { command = nullptr; }

	/**
	 * @brief Create a new ProxyCommand with a \refitem Command pointer
	 * @param command The command to schedule as a proxy
	 */
	explicit ProxyCommand(Command *command) : ProxyCommand([command]() { return command; }) {}

	/**
	 * @brief Initialize the ProxyCommand and putting it into the \refitem CommandScheduler
	 */
	void initialize() override {
		this->command = supplier();
		this->command->schedule();
	}

	/**
	 * @brief Creates the isFinished behavior to track the progress for \refitem Sequence
	 * @return Returns true if the command is no longer scheduled
	 */
	bool isFinished() override { return command == nullptr || !command->scheduled(); }

	/**
	 * @brief Handles ending the task if ProxyCommand is interrupted
	 * @param interrupted Cancels the \refitem Command is cancelled is true
	 */
	void end(bool interrupted) override {
		if (interrupted) {
			command->cancel();
		}
		command = nullptr;
	}

	/**
	 * @brief This \refitem Command has no requirements
	 *
	 * @return This \refitem Command doesn't require the \refitem Subsystem to make sure that it is free after use
	 */
	std::vector<Subsystem *> getRequirements() override { return {}; }

	~ProxyCommand() override = default;
};

inline Command *Command::asProxy() {
	return new ProxyCommand(this);
}

