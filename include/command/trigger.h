#pragma once

class Trigger {
private:
	std::function<bool()> condition;
	EventLoop *eventLoop;

public:
	Trigger(std::function<bool()> condition, EventLoop *event_loop)
		: condition(std::move(condition)),
		  eventLoop(event_loop) {
	}

	explicit Trigger(std::function<bool()> condition)
		: condition(std::move(condition)) {
		eventLoop = CommandScheduler::getEventLoop();
	}

	Trigger *onChange(Command *command) {
		eventLoop->bind(
			[condition = condition, previous = condition(), command]() mutable {
				bool current = condition();

				if (previous != current) {
					command->schedule();
				}

				previous = current;
			});
		return this;
	}

	Trigger *onTrue(Command *command) {
		eventLoop->bind(
			[condition = condition, previous = condition(), command]() mutable {
				bool current = condition();

				if (!previous && current) {
					command->schedule();
				}

				previous = current;
			});
		return this;
	}

	Trigger *onFalse(Command *command) {
		eventLoop->bind(
			[condition = condition, previous = condition(), command]() mutable {
				bool current = condition();

				if (previous && !current) {
					command->schedule();
				}

				previous = current;
			});
		return this;
	}

	Trigger *whileTrue(Command *command) {
		eventLoop->bind(
			[condition = condition, previous = condition(), command]() mutable {
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

	Trigger *whileFalse(Command *command) {
		eventLoop->bind(
			[condition = condition, previous = condition(), command]() mutable {
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

	Trigger *toggleOnTrue(Command *command) {
		eventLoop->bind([condition = condition, previous = condition(),
				command]() mutable {
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

	Trigger *toggleOnFalse(Command *command) {
		eventLoop->bind([condition = condition, previous = condition(),
				command]() mutable {
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
};
