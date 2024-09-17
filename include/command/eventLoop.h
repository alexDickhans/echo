#pragma once

/**
 * @brief Event loops store user-defined bindings to be run every frame. This is used mostly to control the bindings
 * necessary for \refitem Trigger
 */
class EventLoop {
private:
	std::vector<std::function<void()>> bindings;
public:
	/**
	 * Initialize a new empty EventLoop with no bindings
	 */
	EventLoop() = default;

	/**
	 * @brief Initialize the EventLoop with a vector of bindings
	 *
	 * @param bindings Vector storing the bindings to initialize the EventLoop
	 */
	explicit EventLoop(std::vector<std::function<void()>> bindings) : bindings(std::move(bindings)) {}

	/**
	 * @brief Initialize the EventLoop with a initializer list of bindings
	 *
	 * @param bindings Initializer list for new bindings
	 */
	EventLoop(const std::initializer_list<std::function<void()>> bindings) : bindings(bindings) {}

	/**
	 * @brief Poll is run every frame and runs each of the bindings. This is generally run by the CommandScheduler
	 */
	void poll() {
		for (const auto& binding: this->bindings) {
			binding();
		}
	}

	/**
	 * @brief Bind a new command to the EventLoop
	 * @param binding The void function binding to run every frame
	 */
	void bind(const std::function<void()>& binding) {
		bindings.emplace_back(binding);
	}

	/**
	 * @brief Clear all bindings on this event loop
	 */
	void clear() {
		bindings.clear();
	}

	~EventLoop() = default;
};