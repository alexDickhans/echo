#pragma once

#include "config.h"
#include "sensor.h"
#include "utils/utils.h"

class GpsSensor : public Sensor {
private:
	pros::Gps gps;
	Angle sensorAngleOffset;
	Eigen::Vector2f point{};
	double std{0.0};
	bool notInstalled{false};
public:
	GpsSensor(const Angle sensorAngleOffset, pros::Gps gps)
		: gps(std::move(gps)),
		  sensorAngleOffset(sensorAngleOffset) {
	}

	void update() override {
		notInstalled = !gps.is_installed() || gps.get_error() > 0.015;
		auto [x, y] = gps.get_position();

		point = Eigen::Vector2f(-y, x);

		std = gps.get_error() * 8.0;
	}

	std::optional<double> p(const Eigen::Vector3f& X) override {
		if (notInstalled) [[unlikely]] {
			return std::nullopt;
		}

		return cheap_norm_pdf(sqrt(X.x() * point.x() + X.y() * point.y()) / 2.0f) * LOCO_CONFIG::GPS_WEIGHT;
	}

	Angle getAngle() {
		return -gps.get_yaw() * 1_deg - sensorAngleOffset;
	}

	pros::Gps& getGps() {
		return gps;
	}

	~GpsSensor() override = default;
};