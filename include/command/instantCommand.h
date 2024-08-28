#pragma once

class InstantCommand : public FunctionalCommand {
public:
	InstantCommand(const std::function<void()> &on_init, const std::initializer_list<Subsystem *> &requirements)
		: FunctionalCommand(on_init, [] {}, [](bool _) {}, [] { return true; } , requirements) {
	}

	~InstantCommand() override = default;
};
