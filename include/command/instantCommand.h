#pragma once

#include "functionalCommand.h"

/**
 * \refitem Command that runs and initialize function instantly, then ends immediately after first update
 */
class InstantCommand : public FunctionalCommand {
public:
	/**
	 * Create a new InstantCommand
	 *
	 * @param on_init Function to run once upon function start
	 * @param requirements Subsystem requirements for this command
	 */
	InstantCommand(const std::function<void()> &on_init, const std::initializer_list<Subsystem *> &requirements)
		: FunctionalCommand(on_init, [] {}, [](bool _) {}, [] { return true; } , requirements) {
	}

	~InstantCommand() override = default;
};
