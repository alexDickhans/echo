#pragma once

#include "functionalCommand.h"

/**
 * @brief Creates a \refitem FunctionalCommand that runs forever with an onRun function that runs every time execute is
 * run by the \refitem CommandScheduler
 */
class RunCommand : public FunctionalCommand {
public:
	/**
	 * @brief Initializes a RunCommand with the onRun function and necessary \refitem Subsystem requirements
	 *
	 * @param onRun Function to run every frame
	 * @param requirements Requirements that this function will need
	 */
	RunCommand(const std::function<void()>& onRun, const std::initializer_list<Subsystem*> requirements): FunctionalCommand([] {}, onRun, [](bool _) { }, [] () {return false; }, requirements) {
	}
};