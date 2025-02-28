#pragma once

#include "Eigen/Eigen"

#include "feedback/pid.h"
#include "units/units.hpp"

#include "auton.h"
#include "autonomous/autons.h"

namespace CONFIG {
    constexpr double INTAKE_RATIO = 68 / (9 * 3); // 66 chain links
    constexpr float DRIVE_NOISE = 0.05;
    constexpr Angle ANGLE_NOISE = 3_deg;

    constexpr QLength DRIVE_RADIUS = 2.75_in / 2.0;
    constexpr float DRIVE_RATIO = 60.0 / 36.0;
    constexpr float STRING_RATIO = 1.0;
    constexpr double LIFT_RATIO = 18.0/6.0;
    constexpr QLength TRACK_WIDTH = 14_in;
    constexpr size_t NUM_PARTICLES = 250;
    constexpr Angle ANGLE_FINISH_THRESHOLD = 2.0_deg;
    constexpr double ANGLE_DA_FINISH_THRESHOLD = 0.04;
    constexpr double DRIVETRAIN_TUNING_SCALAR = 0.97;

    constexpr double TOP_INTAKE_DEFAULT_TOLERANCE = 0.005;

    constexpr QVelocity MAX_SPEED = 68_in / second;

    inline PID TOP_INTAKE_PID = PID(6.0, 0.0, 4.0);

    inline PID TURN_PID = PID(0.9, 0.0, 6.0);
    inline PID DISTANCE_PID = PID(7.0, 0.00, 0.0);

    constexpr Angle WALL_STAKE_LOAD_HEIGHT = 20_deg;
    constexpr Angle WALL_STAKE_PRIME_HEIGHT = 50_deg;
    constexpr Angle WALL_STAKE_SCORE_HEIGHT = 128_deg;
    constexpr Angle DESCORE_HEIGHT = 140_deg;
    constexpr Angle ALLIANCE_STAKE_SCORE_HEIGHT = 160_deg;

    inline double DRIVETRAIN_FEEDFORWARD(const QVelocity velocity, const QAcceleration accel) {
        return (velocity).getValue() * 0.67 + (accel).getValue() * 0.037 + copysign(0.02, velocity.getValue());
    }

    const Eigen::Vector3f DISTANCE_LEFT_OFFSET((-2.9_in).getValue(), (6.75_in).getValue(), (90_deg).getValue());
    const Eigen::Vector3f DISTANCE_FRONT_OFFSET((7.5_in).getValue(), (-5.25_in).getValue(), (0_deg).getValue());
    const Eigen::Vector3f DISTANCE_RIGHT_OFFSET((-2.9_in).getValue(), (-5.8_in).getValue(), (-90_deg).getValue());
    const Eigen::Vector3f DISTANCE_BACK_OFFSET((-3.1_in).getValue(), (6.5_in).getValue(), (180_deg).getValue());

    constexpr auto AI_VISION_PIXELS_TO_DEGREES = 0.20443037974_deg;

    constexpr float RAMSETE_ZETA = 0.6;
    constexpr float RAMSETE_BETA = 40.0;

    inline Eigen::Matrix3f DEFAULT_DT_COST_Q = Eigen::Matrix3f({{1.0, 0.0, 0.0}, {0.0, 5.0, 0.0}, {0.0, 0.0, 7.0}});
    inline Eigen::Matrix2f DEFAULT_DT_COST_R = Eigen::Matrix2f::Identity();

    inline double K_s = 0.12;

    inline Eigen::RowVector2d DRIVETRAIN_LINEAR_VELOCITY_FF_GOAL{0.553, 0.044};
    inline Eigen::RowVector2d DRIVETRAIN_ANGULAR_VELOCITY_FF_GOAL{0.1560, 0.0204};
    inline Eigen::RowVector2d DRIVETRAIN_LINEAR_VELOCITY_FF_NO_GOAL{0.53, 0.040};
    inline Eigen::RowVector2d DRIVETRAIN_ANGULAR_VELOCITY_FF_NO_GOAL{0.1482, 0.0175};

    constexpr QLength START_STRING_LENGTH = 4.5_in;
    constexpr QLength WINCH_RADIUS = 0.303_in / 2.0;
} // namespace CONFIG
