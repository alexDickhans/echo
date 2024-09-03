#pragma once

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

	static void registerCommand(const std::string& name, Command* command) {
		auto instance = getInstance();

		instance.commands[name] = command;
	}

	static void schedule(const std::string& name) {
		auto instance = getInstance();

		if (instance.commands.contains(name)) {
			CommandScheduler::schedule(instance.commands[name]);
		}
	}
};
