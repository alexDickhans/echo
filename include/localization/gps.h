#pragma once

#include "sensor.h"

class GpsSensor : public Sensor {
private:
	pros::Gps gps;
	Angle sensorAngleOffset;
public:
	GpsSensor(pros::Gps gps, const Angle sensorAngleOffset)
		: gps(std::move(gps)),
		  sensorAngleOffset(sensorAngleOffset) {
	}

	std::optional<double> p(Eigen::Vector3d x) override {
		if (!gps.is_installed()) {
			return std::nullopt;
		}

		// TODO: Determine confidence

	}

	~GpsSensor() override = default;
};