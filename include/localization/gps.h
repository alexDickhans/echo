#pragma once

#include "config.h"
#include "sensor.h"
#include "utils/utils.h"

class GpsSensor : public Sensor {
private:
	pros::Gps gps;
	Angle sensorAngleOffset;
	double x{0.0}, y{0.0};
	bool notInstalled{false};
public:
	GpsSensor(const Angle sensorAngleOffset, pros::Gps gps)
		: gps(std::move(gps)),
		  sensorAngleOffset(sensorAngleOffset) {
	}

	void update() override {
		notInstalled = !gps.is_installed();
		auto [x, y] = gps.get_position();

		this->x = x;
		this->y = y;
	}

	std::optional<double> p(Eigen::Vector3d X) override {
		if (notInstalled) [[unlikely]] {
			return std::nullopt;
		}

		const auto std = gps.get_error() * 4.0;

		const auto point = Eigen::Vector2d(-y, x);
		const auto predicted = Eigen::Vector2d(X.x(), X.y());

		return normal_pdf((point - predicted).norm() / std, 0.0, 1.0) * LOCO_CONFIG::GPS_WEIGHT;
	}

	~GpsSensor() override = default;
};