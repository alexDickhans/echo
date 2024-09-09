#pragma once

#include "functionalCommand.h"

class WaitUntilCommand : public FunctionalCommand {
public:
	WaitUntilCommand(const std::function<bool()> &is_finish)
		: FunctionalCommand([]() {}, []() {}, [](bool _) {}, is_finish, {}) {
	}

	~WaitUntilCommand() override = default;
};

inline Command *Command::until(const std::function<bool()>& isFinish) {
	return new ParallelRaceGroup({new WaitUntilCommand(isFinish), this});
}
