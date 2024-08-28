#pragma once

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

	void initialize() override {
		onInit();
	}

	void execute() override {
		onExecute();
	}

	bool isFinished() override {
		return isFinish();
	}

	void end(const bool interrupted) override {
		onEnd(interrupted);
	}

	std::vector<Subsystem *> getRequirements() override {
		return requirements;
	}
};


