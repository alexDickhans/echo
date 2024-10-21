#pragma once
#include "config.h"

#include "command/command.h"
#include "command/runCommand.h"



class BottomIntake : public Subsystem {
    pros::Motor intakeMotor;

public:
    explicit BottomIntake(pros::Motor intake_motor) :
        intakeMotor(std::move(intake_motor)) {
    }

    void periodic() override {
        // No-op
    }

    void setPct(double pct) const {
        // std::cout << pct << std::endl;
        this->intakeMotor.move_voltage(pct * 12000.0);
    }

    RunCommand *stopIntake() {
        return new RunCommand([this]() { this->setPct(0.0); }, {this});
    }

    RunCommand *movePct(double pct) {
        return new RunCommand([this, pct]() { this->setPct(pct); }, {this});
    }

    ~BottomIntake() override = default;
};
