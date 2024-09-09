#pragma once

#include "config.h"
#include "sensor.h"
#include "config.h"

const std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f>> LINES = {
	{{-1.78308, 0}, {1.78308, 0}},
	{{-1.78308, 1.47828}, {1.78308, 1.47828}},
	{{-1.78308, -1.47828}, {1.78308, -1.47828}},
};

const std::vector<float> LINES_Y = {
	0.0,
	1.47828,
	-1.47828,
};

class LineSensor : public Sensor {
private:
	Eigen::Vector2f sensorOffset;
	pros::adi::LineSensor lineSensor;
	bool measured{false};
public:
	LineSensor(Eigen::Vector2f sensor_offset, pros::adi::LineSensor line_sensor)
		: sensorOffset(std::move(sensor_offset)),
		  lineSensor(std::move(line_sensor)) {
	}

	void update() override {
		measured = this->lineSensor.get_value() < LOCO_CONFIG::LINE_SENSOR_THRESHOLD;
	}

	~LineSensor() override = default;

	std::optional<double> p(const Eigen::Vector3f& x) override {
		Eigen::Vector2f sensor_position = Eigen::Rotation2Df(x.z()) * sensorOffset + x.head<2>();

		auto predictedDistance = 50.0_m;

		for (float lines_y : LINES_Y) {
			predictedDistance = std::min(abs(sensor_position.y() - lines_y) * metre, predictedDistance);
		}

		const auto predicted = predictedDistance < LOCO_CONFIG::LINE_SENSOR_DISTANCE_THRESHOLD;

		if (predicted && measured) {
			return 1.0 * LOCO_CONFIG::LINE_WEIGHT;
		} else if (!predicted && !measured) {
			return 1.0 * LOCO_CONFIG::LINE_WEIGHT;
		} else {
			return 0.4 * LOCO_CONFIG::LINE_WEIGHT;
		}
	}
};