#pragma once

#include "command.h"

class ScheduleCommand : public InstantCommand {
public:
	explicit ScheduleCommand(Command* command)
		: InstantCommand([command]() { command->schedule(); }, {}) {
	}

	~ScheduleCommand() override = default;
};
