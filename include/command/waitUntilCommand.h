#pragma once

#include "functionalCommand.h"

/**
 * @brief WaitUntilCommand creates a command that ends once a condition is finished. This command has no requirements.
 */
class WaitUntilCommand : public FunctionalCommand {
public:
	/**
	 * @brief Create a new WaitUntilCommand object with an isFinish conditional
	 *
	 * @param is_finish The conditional to end with. Once it is true the command will finish
	 */
	explicit WaitUntilCommand(const std::function<bool()> &is_finish)
		: FunctionalCommand([]() {}, []() {}, [](bool _) {}, is_finish, {}) {
	}

	~WaitUntilCommand() override = default;
};

inline Command *Command::until(const std::function<bool()>& isFinish) {
	return new ParallelRaceGroup({new WaitUntilCommand(isFinish), this});
}
