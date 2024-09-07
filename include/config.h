#pragma once

#include "Eigen/Eigen"

#include "units/units.hpp"
#include "feedback/pid.h"

namespace CONFIG {
	constexpr double INTAKE_RATIO = 66.0/96.0; // 66 chain links
	constexpr float DRIVE_NOISE = 0.2;
	constexpr Angle ANGLE_NOISE = 0.05_rad;
	constexpr QLength DRIVE_RADIUS = 3.25_in/2.0;
	constexpr float DRIVE_RATIO = 48.0/36.0;
	constexpr double LIFT_RATIO = 8.0;
	constexpr QLength TRACK_WIDTH = 10_in;
	constexpr size_t NUM_PARTICLES = 1000;
	constexpr Angle ANGLE_FINISH_THRESHOLD = 0.5_deg;
	constexpr double ANGLE_DA_FINISH_THRESHOLD = 0.02;
	constexpr double DRIVETRAIN_TUNING_SCALAR = 75.0/87.9;

	constexpr QVelocity MAX_SPEED = 65_in/second;

	inline PID TURN_PID = PID(1.0, 0.00, 9.0);
	inline PID DISTANCE_PID = PID(18.0, 0.00, 0.0);

	inline double DRIVETRAIN_FEEDFORWARD(const QVelocity velocity, const QAcceleration accel) {
		return (velocity / 85_in/second).getValue() + (accel / (530_in/second/second)).getValue() + copysign(0.08, velocity.getValue());
	}

	const Eigen::Vector2f LINE_SENSOR_1_OFFSET((-1.2_in).getValue(), (-0.3_in).getValue());

	const Eigen::Vector3f DISTANCE_LEFT_OFFSET((5.8_in).getValue(), (-4.2_in).getValue(), (90_deg).getValue());
	const Eigen::Vector3f DISTANCE_BACK_OFFSET((-5.2_in).getValue(), (-5.4_in).getValue(), (180_deg).getValue());
	const Eigen::Vector3f DISTANCE_RIGHT_OFFSET((-5.8_in).getValue(), (-4.2_in).getValue(), (-90_deg).getValue());

	const Eigen::Vector3f GPS_OFFSET((-6_in).getValue(), (-4_in).getValue(), (-90_deg).getValue());
}
