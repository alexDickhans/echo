#pragma once

#include "command/command.h"

class ConditionalCommand : public Command {
private:
	Command* primary, *secondary, *selected;
	std::function<bool()> runPrimary;
public:
	ConditionalCommand(Command *primary, Command *secondary, const std::function<bool()> &run_primary)
		: primary(primary),
		  secondary(secondary),
		  runPrimary(run_primary) {
	}

	void initialize() override {
		if (runPrimary()) {
			selected = primary;
		} else {
			selected = secondary;
		}
		selected->initialize();
	}

	void execute() override {
		selected->execute();
	}

	bool isFinished() override {
		return selected->isFinished();
	}

	void end(bool interrupted) override {
		selected->end(interrupted);
	}

	std::vector<Subsystem *> getRequirements() override {
		if (runPrimary()) {
			return primary->getRequirements();
		}
		return secondary->getRequirements();
	}
};