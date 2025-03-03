#pragma once
#include "config.h"

#include "command/command.h"
#include "command/runCommand.h"

#include "pros/motors.hpp"

class MotorSubsystem : public Subsystem {
    pros::Motor intakeMotor;

public:
    explicit MotorSubsystem(pros::Motor intake_motor) :
        intakeMotor(std::move(intake_motor)) {
        intakeMotor.set_encoder_units_all(pros::MotorEncoderUnits::rotations);
    }

    void periodic() override {
        // No-op
    }

    void setPct(const double pct) {
        this->intakeMotor.move_voltage(pct * 12000.0);
    }

    void setSpeed(double speed) {
        this->intakeMotor.move_velocity(speed);
    }

    RunCommand *stopIntake() {
        return new RunCommand([this]() { this->setPct(0.0); }, {this});
    }

    RunCommand *pctCommand(double pct) {
        return new RunCommand([this, pct]() { this->setPct(pct); }, {this});
    }

    double getTopMotorTemp() const {
        return this->intakeMotor.get_temperature();
    }

    ~MotorSubsystem() override = default;
};
