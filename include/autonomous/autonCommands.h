#pragma once

#include "autons.h"
#include "auton.h"
#include "command/includes.h"
#include "skills.h"
#include "elims.h"

/**
 * Allows easy selection of autonomous routines given a AUTON object, called on initialization to build states
 */
class AutonomousCommands {
public:
    static Command *getAuton() {
        switch (AUTON) {
            case SKILLS:
                return Skills::skills();
            case POS_ELIM:
                return Elims::pos_elim();
            case POS_ELIM_NO_ALLIANCE:
                return Elims::pos_elim_no_alliance();
            case NEG_ELIM:
                return Elims::neg_elim();
            case NEG_ELIM_POLE_TOUCH:
                return Elims::neg_elim_pole_touch();
            default:
                return new InstantCommand([]() { std::cout << "No auton" << std::endl; }, {});
        }
    }
};
