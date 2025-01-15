#pragma once
#include <set>
#include "command.h"

#include <bits/ranges_algo.h>

/**
 * @brief Runs multiple \refitem Command s at once, with the command ending once all individual commands finsh.
 */
class ParallelCommandGroup : public Command {
private:
    std::vector<std::pair<Command *, bool>> commands;

public:
    /**
     * @brief Create a new ParallelCommandGroup given a initializer list of commands
     *
     * @warning No Commands in the parallel can require the same hardware! If this happens the code will immediately
     * abort
     *
     * @param commands Initializer list of \refitem Command s to run in this parallel
     */
    ParallelCommandGroup(const std::initializer_list<Command *> commands) {
        for (auto command: commands) {
            this->commands.emplace_back(command, false);
        }

        auto requirements = this->ParallelCommandGroup::getRequirements();

        const std::set uniqueRequirements(requirements.begin(), requirements.end());

        assert(requirements.size() == uniqueRequirements.size());
    }

    /**
     * Initialize all commands in the ParallelCommandGroup
     */
    void initialize() override {
        for (auto &[command, running]: commands) {
            command->initialize();
            running = true;
        }
    }

    /**
     * Runs all the active commands in the ParallelCommandGroup
     */
    void execute() override {
        for (auto &[command, running]: commands) {
            if (running) {
                command->execute();

                if (command->isFinished()) {
                    running = false;
                    command->end(false);
                }
            }
        }
    }

    /**
     * @brief Returns true if all commands are finished
     *
     * @return True if all commands are finished
     */
    bool isFinished() override {
        return std::ranges::all_of(commands, [](const auto &pair) { return !pair.second; });
    }

    /**
     * @brief Ends all commands with the ones that should be interrupted being interrupted
     *
     * @param interrupted Ends all commands based on if they are done or not
     */
    void end(bool interrupted) override {
        if (interrupted) {
            for (auto &[command, running]: commands) {
                command->end(true);
                running = true;
            }
        }
    }

    /**
     * @brief Gets all the required subsystems for this ParallelCommandGroup
     *
     * @return Set of all requirements of all the command
     */
    std::vector<Subsystem *> getRequirements() override {
        std::vector<Subsystem *> requirements;

        for (auto& pair : commands) {
            auto& command = pair.first;
            for (auto& requirement : command->getRequirements()) {
                requirements.emplace_back(requirement);
            }
        }

        return requirements;
    }
};

inline Command *Command::with(Command *other) { return new ParallelCommandGroup({this, other}); }
