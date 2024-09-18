#pragma once

#include "command.h"

/**
 * @brief Runs multiple \refitem Command s at once, with the command ending once the first command finishes.
 */
class ParallelRaceGroup : public Command {
private:
	std::vector<Command*> commands;
	bool isDone = false;
public:
	/**
	 * @brief Create a new ParallelRaceGroup given a initializer list of commands
	 *
	 * @warning No Commands in the parallel can require the same hardware! If this happens the code will immediately
	 * abort
	 *
	 * @param commands Initializer list of \refitem Command s to run in this parallel
	 */
	ParallelRaceGroup(const std::initializer_list<Command*> commands) : commands(commands) {
		isDone = false;

		auto requirements = this->ParallelRaceGroup::getRequirements();

		const std::set uniqueRequirements(requirements.begin(), requirements.end());

		assert(requirements.size() == uniqueRequirements.size());
	}

	/**
	 * Initialize all commands in the ParallelRaceGroup
	 */
	void initialize() override {
		this->isDone = false;

		for (const auto command : commands) {
			command->initialize();
		}
	}

	/**
	 * Runs all the active commands in the ParallelRaceGroup and check if any are done
	 */
	void execute() override {
		for (auto command : commands) {
			command->execute();

			if (command->isFinished()) {
				this->isDone = true;
				command->end(false);
			}
		}
	}

	/**
	 * @brief Returns true if any commands is finished
	 *
	 * @return True if any commands is finished
	 */
	bool isFinished() override {
		return this->isDone;
	}

	/**
	 * @brief Ends all commands with the ones that should be interrupted being interrupted
	 *
	 * @param interrupted Ends all commands based on if they are done or not
	 */
	void end(bool interrupted) override {
		for (auto command : this->commands) {
			command->end(!command->isFinished());
		}
	}

	/**
	 * @brief Gets all the required subsystems for this ParallelRaceGroup
	 *
	 * @return Set of all requirements of all the command
	 */
	std::vector<Subsystem *> getRequirements() override {
		std::vector<Subsystem *> requirements;

		for (auto command: commands) {
			for (auto requirement : command->getRequirements()) {
				requirements.emplace_back(requirement);
			}
		}

		return requirements;
	}
};

inline Command *Command::race(Command *other) {
	return new ParallelRaceGroup({this, other});
}

