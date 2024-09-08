#pragma once

#include "command.h"

class ParallelRaceGroup : public Command {
private:
	std::vector<Command*> commands;
	bool isDone = false;
public:
	ParallelRaceGroup(const std::initializer_list<Command*> commands) : commands(commands) {
		isDone = false;

		auto requirements = this->ParallelRaceGroup::getRequirements();

		const std::set uniqueRequirements(requirements.begin(), requirements.end());

		assert(requirements.size() == uniqueRequirements.size());
	}

	void initialize() override {
		this->isDone = false;

		for (const auto command : commands) {
			command->initialize();
		}
	}

	void execute() override {
		for (auto command : commands) {
			command->execute();

			if (command->isFinished()) {
				this->isDone = true;
				command->end(false);
			}
		}
	}

	bool isFinished() override {
		return this->isDone;
	}

	void end(bool interrupted) override {
		for (auto command : this->commands) {
			command->end(!command->isFinished());
		}
	}

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

