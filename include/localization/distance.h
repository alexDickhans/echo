#pragma once

#include "sensor.h"

const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> WALLS = {
	{{1.78308, 1.78308}, {1.78308, -1.78308}},
	{{1.78308, -1.78308}, {-1.78308, -1.78308}},
	{{-1.78308, -1.78308}, {-1.78308, 1.78308}},
	{{-1.78308, 1.78308}, {1.78308, 1.78308}},
};

class Distance : public Sensor {
private:
	Eigen::Vector3d sensorOffset;
	pros::Distance distance;

	QLength measured = 0.0;
	bool exit = false;
	QLength std = 0.0;
public:
	Distance(Eigen::Vector3d sensor_offset, pros::Distance distance)
		: sensorOffset(std::move(sensor_offset)),
		  distance(std::move(distance)) {
	}

	void update() override {
		auto measuredMM = distance.get();

		exit = measuredMM == 9999 || distance.get_object_size() < 30;

		measured = measuredMM * millimetre;

		std = 0.20 * measured / (distance.get_confidence() / 64.0);
	}

	[[nodiscard]] std::optional<double> p(Eigen::Vector3d x) override {

		if (exit) {
			return std::nullopt;
		}

		const Eigen::Vector2d v_1 = Eigen::Rotation2Dd(x.z()) * sensorOffset.head<2>() + x.head<2>();
		const Eigen::Vector2d v_2 = Eigen::Rotation2Dd(sensorOffset.z() + x.z()) * Eigen::Vector2d(1.0, 0.0) + v_1;

		auto predicted = 50_m;

		for (const auto & [v_3, v_4] : WALLS) {
			auto t = ((v_1.x() - v_3.x()) * (v_3.y() - v_4.y())
					- (v_1.y() - v_3.y()) * (v_3.x() - v_4.x()))
					/ ((v_1.x() - v_2.x()) * (v_3.y() - v_4.y())
						- (v_1.y() - v_2.y()) * (v_3.x() - v_4.x()));

			if (t > 0.0 && finite(t)) {
				predicted = std::min(predicted, t * metre);
			}
		}

		return normal_pdf(measured.getValue(), predicted.getValue(), std.getValue()) * LOCO_CONFIG::DISTANCE_WEIGHT;
	}

	~Distance() override = default;
};