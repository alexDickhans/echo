#pragma once

#include "Eigen/Eigen"

#include "units/units.hpp"
#include "feedback/pid.h"

namespace CONFIG {
	constexpr double INTAKE_RATIO = 66.0/96.0; // 66 chain links
	constexpr float DRIVE_NOISE = 0.35;
	constexpr Angle ANGLE_NOISE = 8_deg;
	constexpr QLength DRIVE_RADIUS = 3.25_in/2.0;
	constexpr float DRIVE_RATIO = 48.0/36.0;
	constexpr double LIFT_RATIO = 8.0;
	constexpr QLength TRACK_WIDTH = 14_in;
	constexpr size_t NUM_PARTICLES = 500;
	constexpr Angle ANGLE_FINISH_THRESHOLD = 2.0_deg;
	constexpr double ANGLE_DA_FINISH_THRESHOLD = 0.04;
	constexpr double DRIVETRAIN_TUNING_SCALAR = 76.0/87.9;

	constexpr QVelocity MAX_SPEED = 62_in/second;

	inline PID TURN_PID = PID(1.3, 0.00, 9.0);
	inline PID GOAL_PID = PID(1.2, 0.00, 9.0);
	inline PID DISTANCE_PID = PID(36.0, 0.00, 0.0);

	inline double DRIVETRAIN_FEEDFORWARD(const QVelocity velocity, const QAcceleration accel) {
		return (velocity / 85_in/second).getValue() * 1.03 + (accel / (800_in/second/second)).getValue() * 1.05 + copysign(0.05, velocity.getValue()) * 1.03;
	}

	const Eigen::Vector2f LINE_SENSOR_1_OFFSET((-1.2_in).getValue(), (-0.3_in).getValue());

	const Eigen::Vector3f DISTANCE_LEFT_OFFSET((-4.2_in).getValue(), (5.8_in).getValue(), (90_deg).getValue());
	const Eigen::Vector3f DISTANCE_BACK_OFFSET((-5.4_in).getValue(), (-5.2_in).getValue(), (180_deg).getValue());
	const Eigen::Vector3f DISTANCE_RIGHT_OFFSET((-4.2_in).getValue(), (-5.8_in).getValue(), (-90_deg).getValue());

	const Eigen::Vector3f GPS_OFFSET((-6_in).getValue(), (-4_in).getValue(), (-90_deg).getValue());

	constexpr auto AI_VISION_PIXELS_TO_DEGREES = 0.20443037974_deg;

	constexpr float RAMSETE_ZETA = 0.9;
	constexpr float RAMSETE_BETA = 20.0;
}
