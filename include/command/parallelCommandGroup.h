#pragma once
#include <set>
#include "command.h"

class ParallelCommandGroup : public Command {
private:
	std::vector<std::pair<Command *, bool> > commands;

public:
	ParallelCommandGroup(const std::initializer_list<Command *> commands) {
		for (auto command: commands) {
			this->commands.emplace_back(command, false);
		}

		auto requirements = this->ParallelCommandGroup::getRequirements();

		const std::set uniqueRequirements(requirements.begin(), requirements.end());

		assert(requirements.size() == uniqueRequirements.size());
	}

	void initialize() override {
		for (auto &[command, running]: commands) {
			command->initialize();
			running = true;
		}
	}

	void execute() override {
		for (auto &[command, running]: commands) {
			if (running) {
				command->execute();

				if (command->isFinished()) {
					running = false;
					command->end(false);
				}
			}
		}
	}

	bool isFinished() override {
		for (const auto running: commands | std::views::values) {
			if (running) {
				return false;
			}
		}

		return true;
	}

	void end(bool interrupted) override {
		if (interrupted) {
			for (auto &[command, running]: commands) {
				command->end(true);
				running = true;
			}
		}
	}

	std::vector<Subsystem *> getRequirements() override {
		std::vector<Subsystem *> requirements;

		for (auto &command: commands | std::views::keys) {
			for (auto &requirement: command->getRequirements()) {
				requirements.emplace_back(requirement);
			}
		}

		return requirements;
	}
};
