#pragma once

#include "Eigen/Eigen"
#include "units/units.hpp"
#include "sensor.h"

#include <random>
#include <algorithm>

#include "config.h"

template<size_t L>
class ParticleFilter {
private:
	std::array<std::array<float, 2>, L> particles;
	std::array<std::array<float, 2>, L> oldParticles;
	std::array<double, L> weights;

	std::vector<Sensor*> sensors;

	QLength distanceSinceUpdate = 0.0;
	QTime lastUpdateTime = 0.0;

	QLength maxDistanceSinceUpdate = 1_in;
	QTime maxUpdateInterval = 0_s;

	std::function<Angle()> angleFunction;
	std::default_random_engine de;

	std::uniform_real_distribution<> fieldDist{-1.78308, 1.78308};
public:
	explicit ParticleFilter(std::function<Angle()> angle_function)
		: angleFunction(std::move(angle_function)) {
		for (auto&& particle : particles) {
			particle[0] = 0.0;
			particle[1] = 0.0;
		}
	}

	Eigen::Vector3d getPrediction() {
		auto totalX = 0.0;
		auto totalY = 0.0;

		for (const auto & particle : particles) {
			totalX += particle[0];
			totalY += particle[1];
		}

		return {totalX/static_cast<double>(L), totalY/static_cast<double>(L), angleFunction().Convert(radian)};
	}

	std::array<Eigen::Vector3d, L> getParticles() {
		std::array<Eigen::Vector3d, L> particles;

		const Angle angle = angleFunction();

		for (size_t i = 0; i < L; i++) {
			particles[i] = Eigen::Vector3d(this->particles[i][0], this->particles[i][1], angle.getValue());
		}

		return particles;
	}

	void update(const std::function<Eigen::Vector2d()>& predictionFunction) {
		if (!isfinite(angleFunction().getValue())) {
			return;
		}

		const Angle angle = angleFunction();

		for (auto&& particle : particles) {
			auto prediction = predictionFunction();
			particle[0] += prediction.x();
			particle[1] += prediction.y();
		}

		distanceSinceUpdate += predictionFunction().norm();

		// if (distanceSinceUpdate < maxDistanceSinceUpdate && maxUpdateInterval > pros::millis() * millisecond || sensors.empty()) {
		// 	return;
		// }

		for (auto && sensor : this->sensors) {
			sensor->update();
		}

		double totalWeight = 0.0;

		for (size_t i = 0; i < L; i++) {
			weights[i] = 0.0;

			if (outOfField(particles[i])) {
				particles[i][0] = fieldDist(de);
				particles[i][1] = fieldDist(de);
			}

			for (const auto sensor : sensors) {
				if (auto weight = sensor->p(Eigen::Vector3d(particles[i][0], particles[i][1], angle.getValue())); weight.has_value() && isfinite(weight.value())) {
					weights[i] += weight.value();
				}

				weights[i] += LOCO_CONFIG::minWeight;

				totalWeight += weights[i];
			}
		}

		if (totalWeight == 0.0) {
			std::cout << "Warning: Total weight equal to 0" << std::endl;
			return;
		}

		const double avgWeight = totalWeight / static_cast<double>(L);

		std::uniform_real_distribution distribution(0.0, avgWeight);
		const double randWeight = distribution(de);

		oldParticles = particles;

		size_t j = 0;
		auto cumulativeWeight = 0.0;

		for (size_t i = 0; i < L; i++) {
			const auto weight = static_cast<double>(i) * avgWeight + randWeight;

			while (cumulativeWeight < weight) {
				if (j >= weights.size()) {
					break;
				}
				cumulativeWeight += weights[j];
				j++;
			}

			particles[i][0] = oldParticles[j][0];
			particles[i][1] = oldParticles[j][1];
		}

		lastUpdateTime = pros::millis() * millisecond;
		distanceSinceUpdate = 0.0;
	}

	void initNormal(const Eigen::Vector2d& mean, const Eigen::Matrix2d& covariance, const bool flip) {
		for (auto && particle : this->particles) {
			Eigen::Vector2d p = mean + covariance * Eigen::Vector2d::Random();
			particle[0] = p.x();
			particle[1] = p.y() * (flip ? -1.0 : 1.0);
		}
	}

	static bool outOfField(const std::array<float, 2>& vector) {
		return vector[0] > 1.78308 || vector[0] < -1.78308 || vector[1] < -1.78308 || vector[1] > 1.78308;
	}

	void initUniform(QLength minX, QLength minY, QLength maxX, QLength maxY) {
		std::uniform_real_distribution xDistribution(minX.getValue(), maxX.getValue());
		std::uniform_real_distribution yDistribution(minY.getValue(), maxY.getValue());

		for (auto && particle : this->particles) {
			particle[0] = xDistribution(de);
			particle[1] = yDistribution(de);
		}
	}

	void addSensor(Sensor* sensor) {
		this->sensors.emplace_back(sensor);
	}
};
