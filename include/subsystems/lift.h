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

    Angle lastPosition;
    QAngularVelocity velocity;

    QTime lastFree = 0.0;

public:
    explicit LiftSubsystem(const std::initializer_list<int8_t> &motors, const PID &pid) : motor(motors), pid(pid) {
        motor.set_encoder_units_all(pros::MotorEncoderUnits::rotations);
        motor.tare_position_all();
    }

    void periodic() override {
        auto position = this->getPosition();
        if (!voltage.has_value()) {
            const auto command = pid.update(position.Convert(radian));
            motor.move_voltage(command * 12000.0);
        }

        // No-op
        if (abs(motor.get_current_draw()) < 1000) {
            lastFree = pros::millis() * millisecond;
        }

        velocity = (position - lastPosition)/10_ms;

        lastPosition = position;
    }

    void setTarget(Angle angle) {
        pid.setTarget(angle.Convert(radian));
        voltage = std::nullopt;
    }

    void setVoltage(double voltage) {
        this->voltage = voltage;
        motor.move_voltage(voltage * 8000.0);
    }

    Angle getPosition() const {
        return motor.get_position(0) / CONFIG::LIFT_RATIO * revolution;
    }

    FunctionalCommand *positionCommand(Angle angle, Angle threshold = 8_deg) {
        return new FunctionalCommand(
            [this, angle]() { this->setTarget(angle);
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

    Command *zero() {
        return (new FunctionalCommand([this]() { this->setVoltage(-0.2); }, []() {
                                      }, [this](bool _) {
                                      }, [this]() { return this->stalled(300_ms); }, {this}))->withTimeout(2_s)->andThen(
            this->pctCommand(-0.1)->withTimeout(200_ms))->andThen(this->pctCommand(0.0)->withTimeout(200_ms))->andThen(
            new InstantCommand([this]() { this->motor.tare_position(); }, {this}));
    }

    QAngularVelocity getVelocity() const { return velocity; }

    ~LiftSubsystem() override = default;
};
