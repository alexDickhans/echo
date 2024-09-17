#pragma once

#include "command.h"
#include <algorithm>

/**
 * @brief This \refitem Command that runs multiple \refitem Command s in a row.
 */
class Sequence : public Command {
private:
	size_t index = 0;
	std::vector<Command*> commands;
public:
	/**
	 * Creates a new object that runs a series of commands one after another
	 *
	 * @param commands Initializer list for sequence Commands
	 */
	Sequence(const std::initializer_list<Command*> commands) : commands(commands) {

	}

	/**
	 * @brief Initializes the first command
	 */
	void initialize() override {
		index = 0;

		commands[0]->initialize();
	}

	/**
	 * @brief Execute the current command, and step through when it's done
	 */
	void execute() override {
		commands[index]->execute();

		if (commands[index]->isFinished()) {
			commands[index]->end(false);
			index++;
			if (index < commands.size()) {
				commands[index]->initialize();
			}
		}
	}

	/**
	 * Finishes when the last command is finished
	 *
	 * @return Checks if it has completed the last command
	 */
	bool isFinished() override {
		return this->index >= commands.size();
	}

	/**
	 * @brief Ends the correct Command when the Sequence is interrupted
	 *
	 * @param interrupted End the last command if it was interrupted
	 */
	void end(const bool interrupted) override {
		if (index < commands.size()) {
			commands[index]->end(interrupted);
		}
	}

	/**
	 * @brief Returns the requirements the Sequence needs for each step.
	 *
	 * @return Returns a set of all the requirements of all the Commands in the sequence
	 */
	std::vector<Subsystem *> getRequirements() override {
		std::vector<Subsystem*> requirements;

		for (auto* command : commands) {
			for (auto subsystem : command->getRequirements()) {
				if (std::ranges::find(requirements, subsystem) == requirements.end()) {
					requirements.emplace_back(subsystem);
				}
			}
		}

		return requirements;
	}
};

inline Command *Command::andThen(Command *other) {
	return new Sequence({this, other});
}
