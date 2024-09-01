#pragma once

#include <cassert>
#include <unordered_map>
#include "command.h"
#include "subsystem.h"
#include "eventLoop.h"

// Like WPILib's CommandScheduler class
class CommandScheduler {
private:
	std::unordered_map<Subsystem*, Command*> subsystems;
	std::unordered_map<Subsystem*, Command*> requirements;
	std::vector<Command*> scheduledCommands;

	EventLoop eventLoop{};

	CommandScheduler() = default;
public:
	// Singleton pattern
	static CommandScheduler& getInstance() {
		static CommandScheduler instance;
		return instance;
	}

	static void registerSubsystem(Subsystem* subsystem, Command* default_command) {
		CommandScheduler& instance = getInstance();

		// Make sure the subsystem isn't already registered
		assert(instance.subsystems.find(subsystem) == instance.subsystems.end());

		// Make sure the default command isn't null
		assert(default_command != nullptr);

		instance.subsystems[subsystem] = default_command;
	}

	static void schedule(Command* command) {
		CommandScheduler& instance = getInstance();

		// Return if the command is already scheduled
		if (scheduled(command)) {
			return;
		}

		// return if competition is disabled
		if (pros::competition::is_disabled()) {
			return;
		}

		std::vector<Command*> intersection;

		bool all_interruptible = true;
		auto requirements = command->getRequirements();

	    for (auto requirement : instance.requirements) {
	    	if (find(requirements.begin(), requirements.end(), requirement.first) != requirements.end()) {
	            all_interruptible &= requirement.second->getCancelBehavior() == CommandCancelBehavior::CancelRunning;
	    		intersection.push_back(requirement.second);
	        }
	    }

		if (all_interruptible) {
			for (auto intersect : intersection) {
				intersect->end(true);
				std::erase(instance.scheduledCommands, intersect);
			}

			for (auto requirement : requirements) {
				instance.requirements[requirement] = command;
			}

			command->initialize();

			instance.scheduledCommands.push_back(command);
		}
	}

	static std::optional<Command*> getRequiring(Subsystem* subsystem) {
		CommandScheduler& instance = getInstance();

		if (instance.requirements.find(subsystem) != instance.requirements.end()) {
			return instance.requirements[subsystem];
		}

		return std::nullopt;
	}

	static void run() {
		CommandScheduler& instance = getInstance();

		for (const auto key: instance.subsystems | std::views::keys) {
			key->periodic();
		}

		instance.eventLoop.poll();

		for (auto command : instance.scheduledCommands) {
			command->execute();

			if (command->isFinished()) {
				command->end(false);

				for (auto requirement : command->getRequirements()) {
					instance.requirements.erase(requirement);
				}

				std::erase(instance.scheduledCommands, command);
			}
		}

		for (auto [subsystem, command] : instance.subsystems) {
			if (!instance.requirements.contains(subsystem)) {
				schedule(command);
			}
		}
	}

	static bool scheduled(const Command* command) {
		CommandScheduler& instance = getInstance();

		return std::find(instance.scheduledCommands.begin(), instance.scheduledCommands.end(), command) != instance.
		       scheduledCommands.end();
	}

	static EventLoop* getEventLoop() {
		CommandScheduler& instance = getInstance();

		return &instance.eventLoop;
	}

	static void cancel(Command* command) {
		CommandScheduler& instance = getInstance();

		if (scheduled(command)) {
			command->end(true);

			std::erase(instance.scheduledCommands, command);

			for (auto requirement : command->getRequirements()) {
				instance.requirements.erase(requirement);
			}
		}
	}
};

inline void Command::schedule() {
	CommandScheduler::schedule(this);
}

inline void Command::cancel() {
	CommandScheduler::cancel(this);
}

inline bool Command::scheduled() const {
	return CommandScheduler::scheduled(this);
}
