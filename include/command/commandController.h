#pragma once

#include "trigger.h"

class CommandController : public pros::Controller {
public:
	explicit CommandController(const pros::controller_id_e_t id)
		: Controller(id) {
	}

	Trigger* getTrigger(pros::controller_digital_e_t button) {
		return new Trigger([this, button]() { return this->get_digital(button); });
	}
};