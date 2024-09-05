#pragma once

#include "Eigen/Eigen"
#include "units/units.hpp"
#include "sensor.h"

#include <random>

#include "config.h"

template<size_t L>
class ParticleFilter {
private:
	std::array<Eigen::Vector3d, L> particles;

	std::vector<Sensor*> sensors;

	QLength distanceSinceUpdate = 0.0;
	QTime lastUpdateTime = 0.0;

	QLength maxDistanceSinceUpdate = 1_in;
	QTime maxUpdateInterval = 500_s;

	std::function<Angle()> angleFunction;
	std::default_random_engine de;

	std::uniform_real_distribution<> fieldDist{-1.78308, 1.78308};
public:
	explicit ParticleFilter(std::function<Angle()> angle_function)
		: angleFunction(std::move(angle_function)) {
		for (auto&& particle : particles) {
			particle = Eigen::Vector3d(0.0, 0.0, 0.0);
		}
	}

	Eigen::Vector3d getPrediction() {
		auto totalX = 0.0;
		auto totalY = 0.0;

		for (const auto & particle : particles) {
			totalX += particle.x();
			totalY += particle.y();
		}

		return {totalX/static_cast<double>(L), totalY/static_cast<double>(L), angleFunction().Convert(radian)};
	}

	std::array<Eigen::Vector3d, L> getParticles() {
		return particles;
	}

	void update(const std::function<Eigen::Vector2d()>& predictionFunction) {
		if (!isfinite(angleFunction().getValue())) {
			return;
		}

		for (auto& particle : particles) {
			auto prediction = predictionFunction();
			particle = Eigen::Vector3d(particle.x() + prediction.x(), particle.y() + prediction.y(), angleFunction().Convert(radian));
		}

		distanceSinceUpdate += predictionFunction().norm() * metre;

		if (distanceSinceUpdate < maxDistanceSinceUpdate && maxUpdateInterval > pros::millis() * millisecond || sensors.empty()) {
			return;
		}

		std::array<double, L> weights;
		double totalWeight = 0.0;

		for (size_t i = 0; i < particles.size(); i++) {
			weights[i] = 0.0;

			if (outOfField(particles[i])) {
				particles[i].x() = fieldDist(de);
				particles[i].y() = fieldDist(de);
			}

			size_t num_readings = 0;

			for (const auto sensor : sensors) {
				if (auto weight = sensor->p(particles[i]); weight.has_value()) {
					if (isfinite(weight.value())) {
						weights[i] += weight.value();
						num_readings ++;
					}
				}

				weights[i] = weights[i] / static_cast<double>(num_readings) + LOCO_CONFIG::minWeight;

				totalWeight += weights[i];
			}
		}

		if (totalWeight == 0.0) {
			return;
		}

		const double avgWeight = totalWeight / static_cast<double>(L);
		std::uniform_real_distribution distribution(0.0, avgWeight);

		const double randWeight = distribution(de);

		std::array<Eigen::Vector3d, L> oldParticles{particles};

		for (size_t i = 0; i < L; i++) {
			const auto weight = static_cast<double>(i) * avgWeight + randWeight;

			auto weightSum = 0.0;

			size_t j = 0;

			for (; weightSum < weight; j++) {
				if (j >= weights.size()) {
					break;
				}
				weightSum += weights[j];
			}

			particles[i] = oldParticles[j];
		}

		lastUpdateTime = pros::millis() * millisecond;
		distanceSinceUpdate = 0.0;
	}

	void initNormal(const Eigen::Vector3d& mean, const Eigen::Matrix3d& covariance, const bool flip) {
		std::normal_distribution distribution(0.0, 1.0);

		for (auto && particle : this->particles) {
			particle = mean + covariance * Eigen::Vector3d({distribution(de), distribution(de) * (flip ? -1.0 : 1.0), distribution(de)});

			particle.z() = this->angleFunction().getValue();
		}
	}

	static bool outOfField(const Eigen::Vector3d& vector) {
		return vector.x() > 1.78308 || vector.x() < -1.78308 || vector.y() < -1.78308 || vector.y() > 1.78308;
	}

	void initUniform(QLength minX, QLength minY, QLength maxX, QLength maxY) {
		std::uniform_real_distribution xDistribution(minX.getValue(), maxX.getValue());
		std::uniform_real_distribution yDistribution(minY.getValue(), maxY.getValue());

		for (auto && particle : this->particles) {
			particle = Eigen::Vector3d(xDistribution(de), yDistribution(de), angleFunction().getValue());
		}
	}

	void addSensor(Sensor* sensor) {
		this->sensors.emplace_back(sensor);
	}
};
