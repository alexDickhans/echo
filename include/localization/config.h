#pragma once

namespace LOCO_CONFIG {
	constexpr size_t LINE_SENSOR_THRESHOLD = 2000;
	constexpr QLength LINE_SENSOR_DISTANCE_THRESHOLD = 3_in;

	constexpr float DISTANCE_WEIGHT = 1.0;
	constexpr float GPS_WEIGHT = 1.0;
	constexpr float LINE_WEIGHT = 1.0;
}
