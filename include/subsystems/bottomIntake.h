#pragma once
#include "config.h"

#include "command/command.h"
#include "command/runCommand.h"

#include "pros/motors.hpp"

class BottomIntake : public Subsystem {
    pros::Motor intakeMotor;

public:
    explicit BottomIntake(pros::Motor intake_motor) :
        intakeMotor(std::move(intake_motor)) {
        intakeMotor.set_encoder_units_all(pros::MotorEncoderUnits::rotations);
    }

    void periodic() override {
        // No-op
    }

    void setPct(const double pct) {
        // std::cout << pct << std::endl;
        this->intakeMotor.move_voltage(pct * 12000.0);
    }

    void setSpeed(double speed) {
        this->intakeMotor.move_velocity(speed);
    }

    RunCommand *stopIntake() {
        return new RunCommand([this]() { this->setPct(0.0); }, {this});
    }

    RunCommand *movePct(double pct) {
        return new RunCommand([this, pct]() { this->setPct(pct); }, {this});
    }

    RunCommand *speedCommand(double speed) {
        return new RunCommand([this, speed]() { this->setSpeed(speed); }, {this});
    }

    ~BottomIntake() override = default;
};
