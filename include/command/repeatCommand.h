#pragma once

#include "command.h"

/**
 * @brief Makes a \refitem Command repeat each time after it is run
 */
class RepeatCommand : public Command {
private:
	Command* command;
public:
	/**
	 * @brief Create a RepeatCommand with a \refitem Command pointer
	 *
	 * @param command The \refitem Command to make into a RepeatCommand
	 */
	explicit RepeatCommand(Command* command) {
		this->command = command;
	}

	/**
	 * @brief Just initializes the \refitem Command
	 */
	void initialize() override {
		command->initialize();
	}

	/**
	 * @brief Executes the \refitem Command, if it is finished it restarts the \refitem Command
	 */
	void execute() override {
		command->execute();

		if (command->isFinished()) {
			command->end(false);
			command->initialize();
		}
	}

	/**
	 * @brief Ends the \refitem Command with interrupted set to true
	 *
	 * @param interrupted Ignored, if the command is ended it must be interrupted because it is always restarting
	 */
	void end(bool interrupted) override {
		command->end(true);
	}

	/**
	 * @brief Passes on the requirements of the \refitem Command that was passed in
	 *
	 * @return The requirements of the \refitem Command that was passed in
	 */
	std::vector<Subsystem *> getRequirements() override {
		return std::move(command->getRequirements());
	}

	~RepeatCommand() override = default;
};

inline Command *Command::repeatedly() {
	return new RepeatCommand(this);
}

