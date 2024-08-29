#pragma once

namespace CONFIG {
	constexpr double INTAKE_RATIO = 16.5/6.0;
	constexpr double DRIVE_NOISE = 0.1;
	constexpr Angle ANGLE_NOISE = 0.05_rad;
	constexpr QLength DRIVE_RADIUS = 3.25_in/2.0;
	constexpr double DRIVE_RATIO = 4.0; // TODO: Actual value
	constexpr double LIFT_RATIO = 8.0;
	constexpr QLength TRACK_WIDTH = 13_in;
}