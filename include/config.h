#pragma once

#include "Eigen/Eigen"

#include "feedback/pid.h"
#include "units/units.hpp"

#include "auton.h"
#include "autonomous/autons.h"

namespace CONFIG {
    constexpr double INTAKE_RATIO = 67.0 / (10.0 * 3.0); // 66 chain links
    constexpr float DRIVE_NOISE = 0.05;
    constexpr Angle ANGLE_NOISE = 3_deg;

    constexpr QLength DRIVE_RADIUS = 2.75_in / 2.0;
    constexpr QLength ODOM_RADIUS = 2_in / 2.0;
    constexpr float DRIVE_RATIO = 48.0 / 36.0;
    constexpr float STRING_RATIO = 1.0;
    constexpr double LIFT_RATIO = 18.0 / 6.0;
    constexpr QLength TRACK_WIDTH = 11_in;
    constexpr size_t NUM_PARTICLES = 250;
    constexpr Angle ANGLE_FINISH_THRESHOLD = 0.0_deg;
    constexpr double ANGLE_DA_FINISH_THRESHOLD = 0.04;
    constexpr double DRIVETRAIN_TUNING_SCALAR = 0.955;

    constexpr double TOP_INTAKE_DEFAULT_TOLERANCE = 0.005;

    constexpr QVelocity MAX_SPEED = 63_in / second;

    inline PID TOP_INTAKE_PID = PID(6.0, 0.0, 8.0);

    inline PID TURN_PID_NO_GOAL = PID(0.8, 0.0, 6.8);
    inline PID TURN_PID_GOAL = PID(0.88, 0.0, 7.6);
    inline PID DISTANCE_PID = PID(7.0, 0.00, 0.0);

    constexpr Angle LIFT_IDLE_POSITION = 0_deg;
    constexpr Angle WALL_STAKE_LOAD_HEIGHT = 20_deg;
    constexpr Angle WALL_STAKE_PRIME_HEIGHT = 95_deg;
    constexpr Angle DESCORE_HEIGHT = 155_deg;
    constexpr Angle ALLIANCE_STAKE_SCORE_HEIGHT = 185_deg;

    const Eigen::Vector3f DISTANCE_LEFT_OFFSET((4.2_in).getValue(), (5.0_in).getValue(), (90_deg).getValue());
    const Eigen::Vector3f DISTANCE_RIGHT_OFFSET((4.2_in).getValue(), (-5.0_in).getValue(), (-90_deg).getValue());
    const Eigen::Vector3f DISTANCE_FRONT_OFFSET((3.9_in).getValue(), (3.3_in).getValue(), (0_deg).getValue());
    const Eigen::Vector3f DISTANCE_BACK_OFFSET((-5.9_in).getValue(), (-5.2_in).getValue(), (180_deg).getValue());

    constexpr auto AI_VISION_PIXELS_TO_DEGREES = 0.20443037974_deg;

    constexpr float RAMSETE_ZETA = 0.4;
    constexpr float RAMSETE_BETA = 45.0;

    inline Eigen::Matrix3f DEFAULT_DT_COST_Q = Eigen::Matrix3f({{1.0, 0.0, 0.0}, {0.0, 5.0, 0.0}, {0.0, 0.0, 7.0}});
    inline Eigen::Matrix2f DEFAULT_DT_COST_R = Eigen::Matrix2f::Identity();

    inline double K_s = 0.008;

    inline Eigen::RowVector2d DRIVETRAIN_LINEAR_VELOCITY_FF_GOAL{0.550, 0.00275};
    inline Eigen::RowVector2d DRIVETRAIN_ANGULAR_VELOCITY_FF_GOAL{0.0895, 0.0170};
    inline Eigen::RowVector2d DRIVETRAIN_LINEAR_VELOCITY_FF_NO_GOAL{0.546, 0.00272};
    inline Eigen::RowVector2d DRIVETRAIN_ANGULAR_VELOCITY_FF_NO_GOAL{0.0651, 0.0117};

    inline std::pair<double, double> DRIVETRAIN_FEEDFORWARD(const QVelocity velocity, const QAcceleration accel,
                                                            const QAngularVelocity angularVelocity,
                                                            const QAngularAcceleration angularAcceleration) {
        const double uLinear =
            DRIVETRAIN_LINEAR_VELOCITY_FF_NO_GOAL * Eigen::Vector2d(velocity.getValue(), accel.getValue());
        const double uAngular = DRIVETRAIN_ANGULAR_VELOCITY_FF_NO_GOAL *
                                Eigen::Vector2d(angularVelocity.getValue(), angularAcceleration.getValue());

        return {uLinear, uAngular};
    }

    constexpr QLength START_STRING_LENGTH = 0.0;
    constexpr QLength WINCH_RADIUS = 0.303_in / 2.0;
} // namespace CONFIG
