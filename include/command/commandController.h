#pragma once

#include "commandScheduler.h"
#include "trigger.h"

/**
 * @brief Command Controller overrides pros::Controller and makes it better interface with \refitem Trigger and command
 * based code
 *
 * ```c
 * // Initialize CommandController
 * CommandController primary(pros::E_CONTROLLER_MASTER);
 *
 * Command *command;
 *
 * // Register trigger to toggle command on pressing DIGITAL_R2
 * primary.getTrigger(DIGITAL_R2)->toggleOnTrue(command);
 * ```
 */
class CommandController : public pros::Controller {
public:
	/**
	 * @brief Construct a new CommandController
	 *
	 * @param id Master, Partner controller ID
	 */
	explicit CommandController(const pros::controller_id_e_t id) : Controller(id) {}

	/**
	 * @brief Create trigger from button
	 *
	 * @param button The button to create a \refitem Trigger on
	 * @return \refitem Trigger with the boolean as the desired button
	 */
	Trigger *getTrigger(pros::controller_digital_e_t button) {
		return new Trigger([this, button]() { return this->get_digital(button); },
						   CommandScheduler::getTeleopEventLoop());
	}
};
