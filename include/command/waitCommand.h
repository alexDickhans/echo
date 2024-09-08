#pragma once

#include "command.h"
#include "units/units.hpp"

class WaitCommand : public Command {
	QTime startTime;
	QTime duration;
public:
	explicit WaitCommand(const QTime &duration)
		: duration(duration) {
	}

	void initialize() override {
		startTime = pros::millis() * millisecond;
	}

	bool isFinished() override {
		return pros::millis() * millisecond - startTime > duration;
	}

	~WaitCommand() override = default;
};

inline Command *Command::withTimeout(const QTime duration) {
	return new ParallelRaceGroup({new WaitCommand(duration), this});
}

