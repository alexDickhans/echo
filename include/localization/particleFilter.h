#pragma once

#include "Eigen/Eigen"
#include "units/units.hpp"
#include "sensor.h"

#include <random>

template<size_t L>
class ParticleFilter {
private:
	std::array<Eigen::Vector3d, L> particles;

	std::vector<Sensor*> sensors;

	QLength distanceSinceUpdate = 0.0;
	QTime lastUpdateTime = 0.0;

	QLength maxDistanceSinceUpdate = 1_in;
	QTime maxUpdateInterval = 500_ms;

	std::function<Angle()> angleFunction;
	std::default_random_engine de;
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

		if (distanceSinceUpdate < maxDistanceSinceUpdate || maxUpdateInterval > pros::millis() * millisecond) {
			return;
		}

		std::array<double, L> weights;
		double totalWeight = 0.0;

		for (size_t i = 0; i < particles.size(); i++) {
			weights[i] = 0.0;

			size_t num_readings = 0;

			for (auto sensor : sensors) {
				if (auto weight = sensor->p(particles[i]); weight.has_value()) {
					if (isfinite(weight.value())) {
						weights[i] += weight.value();
						num_readings ++;
					}
				}

				weights[i] = weights[i] / static_cast<double>(num_readings);

				totalWeight += weights[i];
			}
		}

		double avgWeight = totalWeight / static_cast<double>(L);
		std::uniform_real_distribution distribution(0.0, avgWeight);

		double randWeight = distribution(de);

		std::array<Eigen::Vector3d, L> newParticles;

		for (size_t i = 0; i < L; i++) {
			auto weight = static_cast<double>(i) * avgWeight + randWeight;

			auto weightSum = 0.0;

			size_t j = 0;

			for (; weightSum < weight; j++) {
				weightSum += weights[j];
			}

			newParticles[i] = particles[j];
		}

		this->particles = newParticles;

		lastUpdateTime = pros::millis() * millisecond;
		distanceSinceUpdate = 0.0;
	}

	void initNormal(const Eigen::Vector3d& mean, const Eigen::Matrix3d& covariance) {
		std::normal_distribution distribution(0.0, 1.0);

		for (auto && particle : this->particles) {
			particle = mean + covariance * Eigen::Vector3d({distribution(de), distribution(de), distribution(de)});

			particle.z() = this->angleFunction().getValue();
		}
	}
};