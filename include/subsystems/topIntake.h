#pragma once

#include "command/instantCommand.h"

#include "config.h"

#include "command/command.h"
#include "command/runCommand.h"
#include "pros/ai_vision.hpp"

#include "pros/distance.hpp"
#include "pros/motor_group.hpp"


enum RingColor_ { Blue = 2, Red = 1, None = 0 } typedef RingColor;

inline pros::aivision_color_s_t RED_COLOR_DESC(1, 196, 80, 127, 40, 0.25);
inline pros::aivision_color_s_t BLUE_COLOR_DESC(2, 52, 73, 125, 40, 0.25);

class TopIntakeSubsystem : public Subsystem {
    pros::MotorGroup intakeMotor;
    pros::AIVision vision;
    pros::Distance intakeDistance;

    double positionOffset = 0.0;

public:
    explicit TopIntakeSubsystem(const std::initializer_list<int8_t> &motors, pros::Distance intakeDistance) : intakeMotor(motors),
        vision(17), intakeDistance(std::move(intakeDistance)) {
        intakeMotor.set_encoder_units_all(pros::MotorEncoderUnits::rotations);
        intakeMotor.set_gearing(pros::MotorGears::green);
        vision.set_color(RED_COLOR_DESC);
        vision.set_color(BLUE_COLOR_DESC);
    }

    void periodic() override {
        // No-op
    }

    void setPosition(double position) const { this->intakeMotor.move_absolute(position * CONFIG::INTAKE_RATIO, 200); }

    void setPct(double pct) const { this->intakeMotor.move_voltage(pct * 12000.0); }

    double getPosition() {
        return (this->intakeMotor.get_position(0) + this->intakeMotor.get_position(1)) / (2 * CONFIG::INTAKE_RATIO) + positionOffset;
    }

    double getVelocity() {
        return (this->intakeMotor.get_actual_velocity(0) + this->intakeMotor.get_actual_velocity(1)) /
               (2 * CONFIG::INTAKE_RATIO);
    }

    bool ringPresent() { return intakeDistance.get() < 90; }

    bool ringPresentEject() { return intakeDistance.get() < 40; }

    RingColor getRingColor() {
        if (vision.get_object_count() != 0) {
            auto largestObject = 0;
            size_t largestSize = 0;

            for (auto object: vision.get_all_objects()) {
                if (size_t size = object.object.color.width * object.object.color.height; size > largestSize) {
                    largestSize = size;
                    largestObject = object.id;
                }
            }

            return static_cast<RingColor>(largestObject);
        }
        return RingColor::None;
    }

    RunCommand *pctCommand(double pct) {
        return new RunCommand([this, pct]() { this->setPct(pct); }, {this});
    }

    RunCommand *controllerCommand(pros::Controller *controller) {
        return new RunCommand(
            [this, controller]() {
                this->setPct(controller->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) / 127.0);
            },
            {this});
    }

    void changeOffset(const double quantity) {
        positionOffset += quantity;
    }

    InstantCommand *adjustOffset(const double quantity) {
        return new InstantCommand([quantity, this]() { this->adjustOffset(quantity); }, {});
    }

    ~TopIntakeSubsystem() override = default;
};
