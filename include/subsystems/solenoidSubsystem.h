#pragma once

#include "command/subsystem.h"

class SolenoidSubsystem : public Subsystem {
    pros::adi::DigitalOut solenoid;

    bool lastValue = false;

public:
    explicit SolenoidSubsystem(pros::adi::DigitalOut solenoid) : solenoid(std::move(solenoid)) {
    }

    void periodic() override {
        // no - op
    }

    void setLevel(const bool value) {
        solenoid.set_value(value);
        lastValue = value;
    }

    RunCommand *levelCommand(bool value) {
        return new RunCommand([this, value]() { this->setLevel(value); }, {this});
    }

    [[nodiscard]] bool getLastValue() const { return lastValue; }

    ~SolenoidSubsystem() override = default;
};
