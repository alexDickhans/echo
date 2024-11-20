#pragma once

#include "Eigen/Eigen"

#include "feedback/pid.h"
#include "units/units.hpp"

namespace CONFIG {
    constexpr double INTAKE_RATIO = 74.0 / 96.0; // 66 chain links
    constexpr float DRIVE_NOISE = 0.35;
    constexpr Angle ANGLE_NOISE = 8_deg;

    constexpr QLength DRIVE_RADIUS = 3.25_in / 2.0;
    constexpr float DRIVE_RATIO = 60.0 / 36.0;
    constexpr double LIFT_RATIO = 5.0;
    constexpr QLength TRACK_WIDTH = 14_in;
    constexpr size_t NUM_PARTICLES = 500;
    constexpr Angle ANGLE_FINISH_THRESHOLD = 2.0_deg;
    constexpr double ANGLE_DA_FINISH_THRESHOLD = 0.04;
    constexpr double DRIVETRAIN_TUNING_SCALAR = 1.0;

    constexpr double TOP_INTAKE_DEFAULT_TOLERANCE = 0.005;

    constexpr QVelocity MAX_SPEED = 60_in / second;

    inline PID TOP_INTAKE_PID = PID(6.0, 0.0, 4.0);

    inline PID TURN_PID = PID(0.9, 0.0, 6.0);
    inline PID DISTANCE_PID = PID(7.0, 0.00, 0.0);

    constexpr Angle WALL_STAKE_LOAD_HEIGHT = 32_deg;
    constexpr Angle WALL_STAKE_SCORE_HEIGHT = 120_deg;

    inline double DRIVETRAIN_FEEDFORWARD(const QVelocity velocity, const QAcceleration accel) {
        return (velocity / 85_in / second).getValue() * 1.03 + (accel / (800_in / second / second)).getValue() * 1.05 +
               copysign(0.05, velocity.getValue()) * 1.03;
    }

    const Eigen::Vector3f DISTANCE_LEFT_OFFSET((-4.2_in).getValue(), (7_in).getValue(), (90_deg).getValue());
    const Eigen::Vector3f DISTANCE_FRONT_OFFSET((7_in).getValue(), (-5_in).getValue(), (180_deg).getValue());
    const Eigen::Vector3f DISTANCE_RIGHT_OFFSET((-4.2_in).getValue(), (-7_in).getValue(), (-90_deg).getValue());

    constexpr auto AI_VISION_PIXELS_TO_DEGREES = 0.20443037974_deg;

    constexpr float RAMSETE_ZETA = 0.0;
    constexpr float RAMSETE_BETA = 1.0;

    Eigen::Matrix3f DEFAULT_DT_COST_Q = Eigen::Matrix3f({{1.0, 0.0, 0.0}, {0.0, 5.0, 0.0}, {0.0, 0.0, 7.0}});
    Eigen::Matrix2f DEFAULT_DT_COST_R = Eigen::Matrix2f::Identity();

    double K_s = 0.12;

    Eigen::RowVector2d DRIVETRAIN_LINEAR_VELOCITY_FF_GOAL{0.63, 0.0503};
    Eigen::RowVector2d DRIVETRAIN_ANGULAR_VELOCITY_FF_GOAL{0.135, 0.006};
    Eigen::RowVector2d DRIVETRAIN_LINEAR_VELOCITY_FF_NO_GOAL = DRIVETRAIN_LINEAR_VELOCITY_FF_GOAL;
    Eigen::RowVector2d DRIVETRAIN_ANGULAR_VELOCITY_FF_NO_GOAL = DRIVETRAIN_ANGULAR_VELOCITY_FF_GOAL;
} // namespace CONFIG
