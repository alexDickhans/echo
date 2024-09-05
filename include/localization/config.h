#pragma once

namespace LOCO_CONFIG {
	constexpr size_t LINE_SENSOR_THRESHOLD = 2000;
	constexpr QLength LINE_SENSOR_DISTANCE_THRESHOLD = 0.5_in;

	constexpr double DISTANCE_WEIGHT = 1.0;
	constexpr double GPS_WEIGHT = 1.0;
	constexpr double LINE_WEIGHT = 1.0;

	constexpr double minWeight = 0.1;
}
