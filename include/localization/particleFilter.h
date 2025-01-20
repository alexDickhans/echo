#pragma once

#include "Eigen/Eigen"
#include "units/units.hpp"
#include "sensor.h"

#include <random>
#include <algorithm>

#include "config.h"

template<size_t L>
class ParticleFilter {
	// Ensure particle count is less than the max of 500 particles
	static_assert(std::less_equal<size_t>()(L, 500), "Too many particles, reduce the number of particles to 500 or less particle to avoid issues with prolonged frame times. ");
private:
	std::array<std::array<float, 2>, L> particles;
	std::array<std::array<float, 2>, L> oldParticles;
	std::array<double, L> weights;

	Eigen::Vector3f prediction{};

	std::vector<Sensor*> sensors;

	QLength distanceSinceUpdate = 0.0;
	QTime lastUpdateTime = 0.0;

	QLength maxDistanceSinceUpdate = 2_in;
	QTime maxUpdateInterval = 2.0_s;

	std::function<Angle()> angleFunction;
	std::ranlux24_base de;

	std::uniform_real_distribution<> fieldDist{-1.78308, 1.78308};
public:
	explicit ParticleFilter(std::function<Angle()> angle_function)
		: angleFunction(std::move(angle_function)) {
		for (auto&& particle : particles) {
			particle[0] = 0.0;
			particle[1] = 0.0;
		}
	}

	Eigen::Vector3f getPrediction() {
		// auto totalX = 0.0;
		// auto totalY = 0.0;
		//
		// for (const auto & particle : particles) {
		// 	totalX += particle[0];
		// 	totalY += particle[1];
		// }
		//
		// return {totalX/static_cast<double>(L), totalY/static_cast<double>(L), angleFunction().Convert(radian)};

		return prediction;
	}

	std::array<Eigen::Vector3f, L> getParticles() {
		std::array<Eigen::Vector3f, L> particles;

		const Angle angle = angleFunction();

		for (size_t i = 0; i < L; i++) {
			particles[i] = Eigen::Vector3f(this->particles[i][0], this->particles[i][1], angle.getValue());
		}

		return particles;
	}

	Eigen::Vector3f getParticle(size_t i) {
        return {this->particles[i][0], this->particles[i][1], angleFunction().getValue()};
    }

    float weightParticle(const Eigen::Vector3f &particle) {
        float totalWeight = 1.0;

        for (const auto sensor: sensors) {
            if (auto weight = sensor->p(particle); weight.has_value() && isfinite(weight.value())) {
                totalWeight = totalWeight * weight.value();
            }
        }

        return totalWeight;
    }

    void updateSensors() {
        for (auto &&sensor: this->sensors) {
            sensor->update();
        }
    }

    void update(const std::function<Eigen::Vector2f()>& predictionFunction) {
		if (!isfinite(angleFunction().getValue())) {
			return;
		}

		auto start = pros::micros();

		const Angle angle = angleFunction();

		for (auto&& particle : particles) {
			auto prediction = predictionFunction();
			particle[0] += prediction.x();
			particle[1] += prediction.y();
		}

		distanceSinceUpdate += predictionFunction().norm();

		if (distanceSinceUpdate < maxDistanceSinceUpdate && maxUpdateInterval > pros::millis() * millisecond - lastUpdateTime) {
			float xSum = 0.0, ySum = 0.0;

			for (size_t i = 0; i < L; i++) {
				xSum += particles[i][0];
				ySum += particles[i][1];
			}

			prediction = Eigen::Vector3f(xSum / static_cast<float>(L), ySum / static_cast<float>(L), angle.getValue());

			return;
		}

		updateSensors();

		double totalWeight = 0.0;

		for (size_t i = 0; i < L; i++) {
			if (outOfField(particles[i])) {
				particles[i][0] = fieldDist(de);
				particles[i][1] = fieldDist(de);
			}

			auto particle = Eigen::Vector3f(particles[i][0], particles[i][1], angle.getValue());

			weights[i] = weightParticle(particle);

			totalWeight = totalWeight + weights[i];
		}

		if (totalWeight == 0.0) {
			std::cout << "Warning: Total weight equal to 0" << std::endl;
			return;
		}

		const double avgWeight = totalWeight / static_cast<double>(L);

		std::uniform_real_distribution distribution(0.0, avgWeight);
		const double randWeight = distribution(de);

		for (size_t i = 0; i < particles.size(); i++) {
			oldParticles[i] = particles[i];
		}

		size_t j = 0;
		auto cumulativeWeight = 0.0;

		float xSum = 0.0, ySum = 0.0;

		for (size_t i = 0; i < L; i++) {
			const auto weight = static_cast<double>(i) * avgWeight + randWeight;

			while (cumulativeWeight < weight) {
				if (j >= weights.size()) {
					break;
				}
				cumulativeWeight += weights[j];
				j++;
			}

			particles[i][0] = oldParticles[j-1][0];
			particles[i][1] = oldParticles[j-1][1];

			xSum += particles[i][0];
			ySum += particles[i][1];
		}

		prediction = Eigen::Vector3f(xSum / static_cast<float>(L), ySum / static_cast<float>(L), angle.getValue());

		lastUpdateTime = pros::millis() * millisecond;
		distanceSinceUpdate = 0.0;
	}

	void initNormal(const Eigen::Vector2f& mean, const Eigen::Matrix2f& covariance, const bool flip) {
		for (auto && particle : this->particles) {
			Eigen::Vector2f p = mean + covariance * Eigen::Vector2f::Random();
			particle[0] = p.x();
			particle[1] = p.y() * (flip ? -1.0 : 1.0);
		}

		prediction.z() = angleFunction().getValue();
		distanceSinceUpdate += 2.0 * distanceSinceUpdate;
	}

	static bool outOfField(const std::array<float, 2>& vector) {
		return vector[0] > 1.78308 || vector[0] < -1.78308 || vector[1] < -1.78308 || vector[1] > 1.78308;
	}

	void initUniform(const QLength minX, const QLength minY, const QLength maxX, const QLength maxY) {
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

	Angle getAngle() {
		return angleFunction();
	}
};
