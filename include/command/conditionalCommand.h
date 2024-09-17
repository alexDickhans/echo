#pragma once

#include "command/command.h"

/**
 * This class creates a conditional command that changes what runs based on a conditional input
 */
class ConditionalCommand : public Command {
private:
	Command* primary, *secondary, *selected{nullptr};
	std::function<bool()> runPrimary;
public:
	/**
	 * @brief Create a new ConditionalCommand
	 *
	 * @param primary The \refitem Command that is run when runPrimary is true on initialization
	 * @param secondary The \refitem Command that is run when runPrimary is false on initialization
	 * @param run_primary Conditional to determine which \refitem Command is run
	 */
	ConditionalCommand(Command *primary, Command *secondary, const std::function<bool()> &run_primary)
		: primary(primary),
		  secondary(secondary),
		  runPrimary(run_primary) {
	}

	/**
	 * @brief Evaluates the conditional and initialized the selected \refitem Command
	 */
	void initialize() override {
		if (runPrimary()) {
			selected = primary;
		} else {
			selected = secondary;
		}
		selected->initialize();
	}

	/**
	 * @brief Runs execute on the selected \refitem Command
	 */
	void execute() override {
		selected->execute();
	}

	/**
	 * @brief Checks if the selected command is finished
	 *
	 * @return selected->isFinished()
	 */
	bool isFinished() override {
		return selected->isFinished();
	}

	/**
	 * Runs the selected function's end command
	 */
	void end(const bool interrupted) override {
		selected->end(interrupted);
	}

	/**
	 * @brief This class only captures the requirements of the running command
	 */
	std::vector<Subsystem *> getRequirements() override {
		if (runPrimary()) {
			return primary->getRequirements();
		}
		return secondary->getRequirements();
	}
};