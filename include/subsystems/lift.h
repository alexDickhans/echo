#pragma once

#include <utility>
#include "command/runCommand.h"

#include "command/subsystem.h"
#include "units/units.hpp"
#include "feedback/pid.h"
#include "config.h"

class LiftSubsystem : public Subsystem {
private:
    pros::MotorGroup motor;

    PID pid;

    std::optional<double> voltage;

    QTime lastFree = 0.0;

public:
    explicit LiftSubsystem(const std::initializer_list<int8_t> &motors, const PID &pid) : motor(motors), pid(pid) {
        motor.set_encoder_units_all(pros::MotorEncoderUnits::rotations);
        motor.tare_position_all();
    }

    void periodic() override {
        if (!voltage.has_value()) {
            const auto command = pid.update(this->getPosition().Convert(radian));
            motor.move_voltage(command * 12000.0);
        }

        // No-op
        if (abs(motor.get_current_draw()) < 600) {
            lastFree = pros::millis() * millisecond;
        }
    }

    void setTarget(Angle angle) {
        pid.setTarget(angle.Convert(radian));
        voltage = std::nullopt;
    }

    void setVoltage(double voltage) {
        this->voltage = voltage;
        motor.move_voltage(voltage * 12000.0);
    }

    Angle getPosition() const {
        return motor.get_position(0) / CONFIG::LIFT_RATIO * revolution;
    }

    FunctionalCommand *positionCommand(Angle angle, Angle threshold = 8_deg) {
        return new FunctionalCommand([]() {
                                     }, [this, angle]() { this->setTarget(angle); }, [](bool _) {
                                     }, [this, threshold, angle]() {
                                         return Qabs(this->getPosition() - angle) < threshold;
                                     }, {this});
    }

    RunCommand *controllerCommand(pros::Controller &controller, pros::controller_analog_e_t channel) {
        Angle targetPosition = 0.0;
        return new RunCommand([this, controller, targetPosition, channel]() mutable {
            targetPosition = targetPosition + controller.get_analog(channel) / 127.0 * 1.0_deg;
            targetPosition = std::clamp(targetPosition, 0_deg, 130_deg);
            this->setTarget(targetPosition);
        }, {this});
    }

    FunctionalCommand *holdPositionCommand() {
        return new FunctionalCommand([this]() {
                                         this->setTarget(
                                             this->getPosition() + 5 *
                                             degree);
                                     }, []() {
                                     }, [](bool _) {
                                     }, []() { return false; }, {this});
    }

    RunCommand *pctCommand(double voltage) {
        return new RunCommand([this, voltage]() mutable { this->setVoltage(voltage); }, {this});
    }

    double getTopMotorTemp() const {
        return this->motor.get_temperature();
    }

    bool stalled(QTime duration) const {
        return pros::millis() * 1_ms - lastFree > duration;
    }

    FunctionalCommand *zero() {
        return new FunctionalCommand([this]() { this->setVoltage(1.0); }, []() {
                                     }, [](bool _) {
                                     }, [this]() { return this->stalled(300_ms); }, {this});
    }

    ~LiftSubsystem() override = default;
};
