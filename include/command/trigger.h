#pragma once

#include "commandScheduler.h"
#include "eventLoop.h"

/**
 * This class allows for easy triggering of Commands based on boolean inputs
 */
class Trigger {
private:
	std::function<bool()> condition;
	EventLoop *eventLoop;

public:
	/**
	 * @brief Create a Trigger with a specified condition and \refitem EventLoop
	 *
	 * @param condition The condition for the Trigger
	 * @param event_loop The \refitem EventLoop for the condition to run on
	 */
	Trigger(std::function<bool()> condition, EventLoop *event_loop) :
		condition(std::move(condition)), eventLoop(event_loop) {}

	/**
	 * @brief Create a Trigger with a specified condition and the default \refitem CommandScheduler \refitem EventLoop
	 *
	 * @param condition The condition for the Trigger
	 */
	explicit Trigger(std::function<bool()> condition) : condition(std::move(condition)) {
		eventLoop = CommandScheduler::getEventLoop();
	}

	explicit Trigger(Command* command) : Trigger([command] () {return command->scheduled(); }) {

	}

	/**
	 * @brief Binds a function to the \refitem EventLoop checks the condition to see if it has changed. Upon change the
	 * \refitem Command is scheduled
	 *
	 * @param command The command to run when the condition changes
	 * @return Trigger to allow for easy method chaining
	 */
	Trigger *onChange(Command *command) {
		eventLoop->bind([condition = condition, previous = condition(), command]() mutable {
			bool current = condition();

			if (previous != current) {
				command->schedule();
			}

			previous = current;
		});
		return this;
	}

	/**
	 * @brief Binds a function to the \refitem EventLoop checks the condition to see if it has changed from false to
	 * true. Upon change the \refitem Command is scheduled
	 *
	 * @param command The command to run when the condition changes from false to true
	 * @return Trigger to allow for easy method chaining
	 */
	Trigger *onTrue(Command *command) {
		eventLoop->bind([condition = condition, previous = condition(), command]() mutable {
			bool current = condition();

			if (!previous && current) {
				command->schedule();
			}

			previous = current;
		});
		return this;
	}

	/**
	 * @brief Binds a function to the \refitem EventLoop checks the condition to see if it has changed from true to
	 * false. Upon change the \refitem Command is scheduled
	 *
	 * @param command The command to run when the condition changes from true to false
	 * @return Trigger to allow for easy method chaining
	 */
	Trigger *onFalse(Command *command) {
		eventLoop->bind([condition = condition, previous = condition(), command]() mutable {
			bool current = condition();

			if (previous && !current) {
				command->schedule();
			}

			previous = current;
		});
		return this;
	}

	/**
	 * @brief Binds a function to the \refitem EventLoop checks the condition to see if it has changed from false to
	 * true. Upon change the \refitem Command is scheduled. When the Trigger detects the condition has changed from
	 * true to false, the Command is cancelled. It is not rescheduled when the command finishes (If this behavior is
	 * desired use a \refitem RepeatCommand)
	 *
	 * @param command The command to run when the condition is true
	 * @return Trigger to allow for easy method chaining
	 */
	Trigger *whileTrue(Command *command) {
		eventLoop->bind([condition = condition, previous = condition(), command]() mutable {
			bool current = condition();

			if (!previous && current) {
				command->schedule();
			} else if (previous && !current) {
				command->cancel();
			}

			previous = current;
		});
		return this;
	}

	/**
	 * @brief Binds a function to the \refitem EventLoop checks the condition to see if it has changed from true to
	 * false. Upon change the \refitem Command is scheduled. When the Trigger detects the condition has changed from
	 * false to true, the Command is cancelled. It is not rescheduled when the command finishes (If this behavior is
	 * desired use a \refitem RepeatCommand)
	 *
	 * @param command The command to run when the condition is false
	 * @return Trigger to allow for easy method chaining
	 */
	Trigger *whileFalse(Command *command) {
		eventLoop->bind([condition = condition, previous = condition(), command]() mutable {
			bool current = condition();

			if (previous && !current) {
				command->schedule();
			} else if (!previous && current) {
				command->cancel();
			}

			previous = current;
		});
		return this;
	}

	/**
	 * @brief Binds a function to the \refitem EventLoop checks the condition to see if it has changed from false to
	 * true. Upon change the \refitem Command is scheduled if it is not already scheduled, and cancelled otherwise
	 *
	 * @param command The command to toggle when the condition changes to true
	 * @return Trigger to allow for easy method chaining
	 */
	Trigger *toggleOnTrue(Command *command) {
		eventLoop->bind([condition = condition, previous = condition(), command]() mutable {
			bool current = condition();

			if (!previous && current) {
				if (command->scheduled()) {
					command->cancel();
				} else {
					command->schedule();
				}
			}

			previous = current;
		});
		return this;
	}

	/**
	 * @brief Binds a function to the \refitem EventLoop checks the condition to see if it has changed from true to
	 * false. Upon change the \refitem Command is scheduled if it is not already scheduled, and cancelled otherwise
	 *
	 * @param command The command to toggle when the condition changes to false
	 * @return Trigger to allow for easy method chaining
	 */
	Trigger *toggleOnFalse(Command *command) {
		eventLoop->bind([condition = condition, previous = condition(), command]() mutable {
			bool current = condition();

			if (previous && !current) {
				if (command->scheduled()) {
					command->cancel();
				} else {
					command->schedule();
				}
			}

			previous = current;
		});
		return this;
	}

	Trigger* andOther(Trigger* trigger) const {
		return new Trigger([trigger, this] () {return trigger->condition() && this->condition();});
	}

	Trigger* orOther(Trigger* trigger) const {
		return new Trigger([trigger, this] () {return trigger->condition() || this->condition();});
	}

	Trigger* negate() const {
		return new Trigger([this] () {return !this->condition();});
	}
};
