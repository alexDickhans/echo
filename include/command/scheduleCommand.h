#pragma once

#include "command.h"
#include "instantCommand.h"

/**
 * @brief This \refitem InstantCommand schedules a \refitem Command on object initialization
 */
class ScheduleCommand : public InstantCommand {
public:
	/**
	 * @brief Creates a \refitem InstantCommand that schedules a \refitem Command on initalization
	 * @param command The \refitem Command to schedule on initialization
	 */
	explicit ScheduleCommand(Command* command)
		: InstantCommand([command]() { command->schedule(); }, {}) {
	}

	~ScheduleCommand() override = default;
};
