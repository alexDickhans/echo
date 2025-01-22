#pragma once

#include "command/subsystem.h"

class LEDArray : public Subsystem {
    pros::adi::Led led;

public:
    explicit LEDArray(pros::adi::Led led) : led(std::move(led)) {
    }

    void periodic() override {
        // no - op
    }

    void setColorAll(const uint32_t value) {
        led.set_all(value);
    }

    void setRepeatAll(const std::vector<uint32_t> &values) {
        for (uint32_t i = 0; i < led.length(); i++) {
            led[i] = values[i % values.size()];
        }

        std::cout << "update leds" << std::endl;

        led.update();
    }

    FunctionalCommand *colorCommand(const uint32_t value) {
        return new FunctionalCommand([this, value]() { this->setColorAll(value); }, [] {
                                     }, [](bool _) {
                                     }, [] { return false; }, {this});
    }

    FunctionalCommand *repeatColor(const std::vector<uint32_t> &values) {
        return new FunctionalCommand([this, values]() { this->setRepeatAll(values); }, [] {
                                     }, [](bool _) {
                                     }, [] { return false; }, {this});
    }

    ~LEDArray() override = default;
};
