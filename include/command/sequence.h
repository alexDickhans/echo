#pragma once

#include "command.h"

class Sequence : public Command {
private:
	size_t index = 0;
	std::vector<Command*> commands;
public:
	Sequence(const std::initializer_list<Command*> commands) : commands(commands) {

	}

	void initialize() override {
		index = 0;

		commands[0]->initialize();
	}

	void execute() override {
		commands[index]->execute();

		if (commands[index]->isFinished()) {
			commands[index]->end(false);
			index++;
			commands[index]->initialize();
		}
	}

	bool isFinished() override {
		return index >= commands.size();
	}

	void end(bool interrupted) override {
		if (index < commands.size()) {
			commands[index]->end(interrupted);
		}
	}

	std::vector<Subsystem *> getRequirements() override {
		std::vector<Subsystem*> requirements;

		for (auto* command : commands) {
			for (auto subsystem : command->getRequirements()) {
				requirements.emplace_back(subsystem);
			}
		}

		return requirements;
	}
};