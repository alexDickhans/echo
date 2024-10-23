#pragma once

#include "autons.h"
#include "auton.h"
#include "command/includes.h"
#include "skills.h"
#include "awp.h"
#include "elims.h"

class AutonomousCommands {
public:
    static Command *getAuton() {
        switch (AUTON) {
            case SKILLS:
                return Skills::skills();
            case AWP:
                return AWP::awp();
            case AWP_PUSH:
                return AWP::push_awp();
            case POS_ELIM:
                return Elims::pos_elim();
            case NEG_ELIM:
                return Elims::neg_elim();
            default:
                return new InstantCommand([]() { std::cout << "No auton" << std::endl; }, {});
        }
    }
};
