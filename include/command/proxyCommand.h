#pragma once

#include "command.h"

class ProxyCommand : public Command {
private:
	Command *command;
	std::function<Command *()> supplier;

public:
	explicit ProxyCommand(const std::function<Command *()> &supplier) : supplier(supplier) { command = nullptr; }

	explicit ProxyCommand(Command *command) : ProxyCommand([command]() { return command; }) {}

	void initialize() override {
		this->command = supplier();
		this->command->schedule();
	}

	bool isFinished() override { return command == nullptr || !command->scheduled(); }

	void end(bool interrupted) override {
		if (interrupted) {
			command->cancel();
		}
		command = nullptr;
	}

	std::vector<Subsystem *> getRequirements() override { return {}; }

	~ProxyCommand() override = default;
};

inline Command *Command::asProxy() {
	return new ProxyCommand(this);
}

