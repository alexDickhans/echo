#pragma once

#include <string>
#include "command/command.h"
#include "command/commandScheduler.h"

class PathCommands {
private:
	std::unordered_map<std::string, Command*> commands;

	PathCommands() = default;
public:
	// Singleton pattern
	static PathCommands& getInstance() {
		static PathCommands instance;
		return instance;
	}

	PathCommands(PathCommands& other) = delete;

	static void registerCommand(const std::string& name, Command* command) {
		PathCommands& instance = getInstance();

		instance.commands.insert_or_assign(name, command);
	}

	static void schedule(const std::string& name) {
		PathCommands& instance = getInstance();

		if (instance.commands.contains(name)) {
			CommandScheduler::schedule(instance.commands[name]);
		}
	}
};
