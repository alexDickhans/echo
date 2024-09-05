#pragma once

#include "config.h"
#include "sensor.h"
#include "utils/utils.h"

class GpsSensor : public Sensor {
private:
	pros::Gps gps;
	Angle sensorAngleOffset;
public:
	GpsSensor(const Angle sensorAngleOffset, pros::Gps gps)
		: gps(std::move(gps)),
		  sensorAngleOffset(sensorAngleOffset) {
	}

	std::optional<double> p(Eigen::Vector3d X) override {
		if (!gps.is_installed()) {
			return std::nullopt;
		}

		// TODO: Flag stuff

		auto [x, y] = gps.get_position();

		const auto std = gps.get_error() * 4.0;

		const auto point = Eigen::Vector2d(-y, x);
		const auto predicted = Eigen::Vector2d(X.x(), X.y());

		return normal_pdf((point - predicted).norm() / std, 0.0, 1.0) * LOCO_CONFIG::GPS_WEIGHT;
	}

	~GpsSensor() override = default;
};