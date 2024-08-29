#pragma once
#include "command.h"

class RepeatCommand : public Command {
private:
	Command* command;
public:
	RepeatCommand(Command* command) {
		this->command = command;
	}

	void initialize() override {
		command->initialize();
	}

	void execute() override {
		command->execute();

		if (command->isFinished()) {
			command->end(false);
			command->initialize();
		}
	}

	void end(bool interrupted) override {
		command->end(true);
	}

	std::vector<Subsystem *> getRequirements() override {
		return std::move(command->getRequirements());
	}

	~RepeatCommand() override = default;
};
