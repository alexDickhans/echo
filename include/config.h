#pragma once

namespace CONFIG {
	constexpr double INTAKE_RATIO = 66.0/24.0; // 66 chain links
	constexpr double DRIVE_NOISE = 0.1;
	constexpr Angle ANGLE_NOISE = 0.05_rad;
	constexpr QLength DRIVE_RADIUS = 3.25_in/2.0;
	constexpr double DRIVE_RATIO = 36.0/48.0;
	constexpr double LIFT_RATIO = 8.0;
	constexpr QLength TRACK_WIDTH = 13_in;
	constexpr size_t NUM_PARTICLES = 100;

	constexpr QVelocity MAX_SPEED = 65_in/second;

	inline PID* TURN_PID = new PID(0.0, 0.0, 0.0);

	inline double DRIVETRAIN_FEEDFORWARD(const QVelocity velocity, const QAcceleration _accel) {
		return (velocity / MAX_SPEED).getValue();
	}
}
