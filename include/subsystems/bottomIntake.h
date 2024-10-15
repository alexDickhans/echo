#pragma once
#include "config.h"

#include "command/command.h"
#include "command/runCommand.h"
#include "vex/v5_vcs.h"

enum RingColor_ { Blue = 2, Red = 1, None = 0 } typedef RingColor;

inline vex::aivision::colordesc RED_COLOR_DESC(1, 255, 0, 0, 50, 0.61);
inline vex::aivision::colordesc BLUE_COLOR_DESC(2, 0, 0, 255, 50, 0.61);

class BottomIntake : public Subsystem {
    pros::Motor intakeMotor;
    vex::aivision vision;

public:
    explicit BottomIntake(pros::Motor intake_motor) :
        intakeMotor(std::move(intake_motor)), vision(12, RED_COLOR_DESC, BLUE_COLOR_DESC) {
        vision.colorDetection(true, true);
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

    RingColor getRingColor() {
        vision.takeSnapshot(vex::aivision::ALL_COLORS);

        return static_cast<RingColor>(vision.largestObject.id);
    }

    ~BottomIntake() override = default;
};
