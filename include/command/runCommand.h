#pragma once

class RunCommand : public FunctionalCommand {
public:
	RunCommand(const std::function<void()>& onRun, const std::initializer_list<Subsystem*> requirements): FunctionalCommand([] {}, onRun, [](bool _) { }, [] () {return false; }, requirements) {
	}
};