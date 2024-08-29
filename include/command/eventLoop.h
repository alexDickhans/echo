#pragma once

class EventLoop {
private:
	std::vector<std::function<void()>> bindings;
public:
	EventLoop() = default;

	void poll() {
		for (const auto& binding: this->bindings) {
			binding();
		}
	}

	void bind(const std::function<void()>& binding) {
		bindings.emplace_back(binding);
	}

	void clear() {
		bindings.clear();
	}

	~EventLoop() = default;
};