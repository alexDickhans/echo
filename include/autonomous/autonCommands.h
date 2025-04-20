#pragma once

#include "autons.h"
#include "auton.h"
#include "command/includes.h"
#include "skills.h"
#include "negative.h"
#include "positive.h"

/**
 * Allows easy selection of autonomous routines given a AUTON object, called on initialization to build states
 */
class AutonomousCommands {
public:
    static Command *getAuton() {
        switch (AUTON) {
            case SKILLS:
                return Skills::skills();
            case N_1_6:
                return Negative::n_1_6();
            case N_6:
            case N_1_5_POLE:
            case P_1_1_2:
            case P_1_3ALLIANCE:
            case P_1_3GOAL:
            case P_4:
            default:
                return new InstantCommand([]() { std::cout << "No auton" << std::endl; }, {});
        }
    }
};
