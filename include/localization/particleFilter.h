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
	ParticleFilter() = default;

	Eigen::Vector3d getPrediction() {
		auto totalX = 0.0;
		auto totalY = 0.0;

		for (const auto & particle : particles) {
			totalX += particle.x();
			totalY += particle.y();
		}

		return {totalX/static_cast<double>(L), totalY/static_cast<double>(L), angleFunction().Convert(radian)};
	}

	void update(const std::function<Eigen::Vector2d()>& predictionFunction) {
		for (auto& particle : particles) {
			auto prediction = predictionFunction();
			particle = Eigen::Vector3d(particle.x + prediction.x, particle.y + prediction.y, angleFunction().Convert(radian));
		}

		distanceSinceUpdate += predictionFunction().norm() * metre;

		if (distanceSinceUpdate < maxDistanceSinceUpdate || maxUpdateInterval > pros::millis() * millisecond) {
			return;
		}

		std::array<double, L> weights;
		double totalWeight = 0.0;

		for (size_t i = 0; i < particles.size(); i++) {
			weights[i] = 0.0;

			// TODO: if sensor produces none, don't make it weigh poorly

			for (auto sensor : sensors) {
				if (auto weight = sensor->p(particles[i]); weight.has_value()) {
					if (isfinite(weight.value())) {
						weights[i] += weight;
						totalWeight += weight;
					}
				}
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
};