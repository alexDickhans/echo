#pragma once

#include "autons.h"
#include "auton.h"
#include "command/includes.h"
#include "skills.h"
#include "awp.h"

class AutonomousCommands {
public:
    static Command *getAuton() {
        switch (AUTON) {
            case SKILLS:
                return Skills::skills();
            case AWP:
                return AWP::awp(ALLIANCE == BLUE);
            case AWP_PUSH:
                return AWP::push_awp(ALLIANCE == BLUE);
            default:
                return new InstantCommand([]() { std::cout << "No auton" << std::endl; }, {});
        }
    }
};
