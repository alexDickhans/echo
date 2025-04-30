#pragma once

#include "command/instantCommand.h"

#include "config.h"

#include "command/command.h"
#include "command/runCommand.h"

#include "pros/distance.hpp"
#include "pros/motor_group.hpp"

#include "pros/optical.hpp"


enum RingColor_ { Blue = 2, Red = 1, None = 0 } typedef RingColor;

class TopIntakeSubsystem : public Subsystem {
    pros::Motor intakeMotor;
    pros::Optical optical;

    double positionOffset = 0.0;

    QTime lastFree = 0.0;
    RingColor ringColor = None;

public:
    explicit TopIntakeSubsystem(const pros::Motor &motors, pros::Optical optical) : intakeMotor(motors),
        optical(optical) {
        intakeMotor.set_encoder_units_all(pros::MotorEncoderUnits::rotations);
        intakeMotor.set_gearing(pros::MotorGears::blue);
        this->optical.set_integration_time(10);
        this->optical.set_led_pwm(255);
    }

    bool visionConnected() {
        return this->optical.is_installed();
    }

    void periodic() override {
        // No-op
        if (abs(intakeMotor.get_current_draw()) < 1700) {
            lastFree = pros::millis() * millisecond;
        }
        ringColor = updateRingColor();
    }

    void setPosition(double position) const { this->intakeMotor.move_absolute(position * CONFIG::INTAKE_RATIO, 200); }

    void setPct(double pct) const {
        this->intakeMotor.move_voltage(pct * 12000.0);
    }

    double getPosition() {
        return this->intakeMotor.get_position(0) / CONFIG::INTAKE_RATIO + positionOffset;
    }

    double getVelocity() {
        return (this->intakeMotor.get_actual_velocity(0) + this->intakeMotor.get_actual_velocity(1)) /
               (2 * CONFIG::INTAKE_RATIO);
    }

    RingColor getRing() {
        return ringColor;
    }

    RingColor updateRingColor() {
        if (optical.get_proximity() > 150) {
            return optical.get_hue() > 180 && optical.get_hue() < 270 ? RingColor::Blue : RingColor::Red;
        }
        return RingColor::None;
    }

    FunctionalCommand *pctCommand(double pct) {
        return new FunctionalCommand([this, pct]() { this->setPct(pct); },
                                     [this]() {  }, [](bool _) {
                                     }, []() { return false; }, {this});
    }

    RunCommand *controllerCommand(pros::Controller *controller) {
        return new RunCommand(
            [this, controller]() {
                this->setPct(controller->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) / 127.0);
            },
            {this});
    }

    bool stalled(QTime duration) const {
        return pros::millis() * 1_ms - lastFree > duration;
    }

    void changeOffset(const double quantity) {
        positionOffset += quantity;
    }

    InstantCommand *adjustOffset(const double quantity) {
        return new InstantCommand([quantity, this]() { this->adjustOffset(quantity); }, {});
    }

    double getTopMotorTemp() const {
        return this->intakeMotor.get_temperature();
    }

    ~TopIntakeSubsystem() override = default;
};
