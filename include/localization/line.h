#pragma once

#include "config.h"
#include "sensor.h"
#include "config.h"

const std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f>> LINES = {
	{{-1.78308, 0}, {1.78308, 0}},
	{{-1.78308, 1.47828}, {1.78308, 1.47828}},
	{{-1.78308, -1.47828}, {1.78308, -1.47828}},
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

		for (auto [fst, snd] : LINES) {
			predictedDistance = std::min(Qabs(((fst.y() - snd.y()) * sensor_position.y()
				- (fst.x() - snd.x()) * sensor_position.x()
				+ snd.x() * fst.y()
				- snd.y() * fst.x())
				/ (fst - snd).norm() * metre), predictedDistance);
		}

		auto predicted = Qabs(predictedDistance) < LOCO_CONFIG::LINE_SENSOR_DISTANCE_THRESHOLD;

		if (predicted && measured) {
			return 1.0 * LOCO_CONFIG::LINE_WEIGHT;
		} else if (!predicted && !measured) {
			return 1.0 * LOCO_CONFIG::LINE_WEIGHT;
		} else if (predicted) {
			return 0.9 * LOCO_CONFIG::LINE_WEIGHT;
		} else {
			return 0.0 * LOCO_CONFIG::LINE_WEIGHT;
		}
	}
};