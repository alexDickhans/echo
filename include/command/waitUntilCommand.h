#pragma once

#include "functionalCommand.h"

class WaitUntilCommand : public FunctionalCommand {
public:
	WaitUntilCommand(const std::function<bool()> &is_finish)
		: FunctionalCommand([]() {}, []() {}, [](bool _) {}, is_finish, {}) {
	}

	~WaitUntilCommand() override = default;
};