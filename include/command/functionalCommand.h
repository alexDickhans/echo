#pragma once

#include "command.h"

class FunctionalCommand : public Command {
private:

	std::function<void()> onInit;
	std::function<void()> onExecute;
	std::function<void(bool)> onEnd;
	std::function<bool()> isFinish;

	std::vector<Subsystem*> requirements;
public:
	FunctionalCommand(std::function<void()> on_init, std::function<void()> on_execute, std::function<void(bool)> on_end,
		std::function<bool()> is_finish, const std::initializer_list<Subsystem*> requirements)
		: onInit(std::move(on_init)),
		  onExecute(std::move(on_execute)),
		  onEnd(std::move(on_end)),
		  isFinish(std::move(is_finish)),
		  requirements(requirements) {
	}

	/**
	 * @brief Runs the user-defined initializer function
	 */
	void initialize() override {
		onInit();
	}

	/**
	 * @brief Runs the user-defined execute function
	 */
	void execute() override {
		onExecute();
	}

	/**
	 * @brief Uses the user-defined function for isFinished
	 *
	 * @return The boolean result from the user defined function
	 */
	bool isFinished() override {
		return isFinish();
	}

	/**
	 * @brief Runs the user-defined end function
	 *
	 * @param interrupted Passes on the interrupted parameter to the user-defined function
	 */
	void end(const bool interrupted) override {
		onEnd(interrupted);
	}

	/**
	 * @brief Returns requirements for the subsystem
	 *
	 * @return User-defined requirements for the subsystem
	 */
	std::vector<Subsystem *> getRequirements() override {
		return requirements;
	}

	/**
	 * Default destructor
	 */
	virtual ~FunctionalCommand() = default;
};


