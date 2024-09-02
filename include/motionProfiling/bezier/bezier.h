#pragma once

#include "json/json.hpp"

#include "Eigen/Eigen"

constexpr size_t BEZIER_POINT_DENSITY = 50;

class Bezier {
private:
	Eigen::Vector2d a, b, c, d;
	bool reversed{};
	bool stopBegin{};
	std::vector<double> distance, t;

	static Eigen::Vector2d toVector(const json11::Json &json) {
		return {json["x"].number_value(), json["y"].number_value()};
	}

public:
	QVelocity velocity;
	QAcceleration acceleration;

	Bezier(Eigen::Vector2d a, Eigen::Vector2d b, Eigen::Vector2d c, Eigen::Vector2d d, const bool reversed,
	       const bool stopBegin,
	       const QVelocity velocity, const QAcceleration acceleration)
		: a(std::move(a)),
		  b(std::move(b)),
		  c(std::move(c)),
		  d(std::move(d)),
		  reversed(reversed),
		  velocity(velocity),
		  acceleration(acceleration),
		  stopBegin(stopBegin) {
		calculate();
	}

	explicit Bezier(const json11::Json &json) : Bezier(toVector(json["path"][0]), toVector(json["path"][1]),
	                                                   toVector(json["path"][2]), toVector(json["path"][3]),
	                                                   json["inverted"].bool_value(), json["stop_end"].bool_value(),
	                                                   json["constraints"]["velocity"].number_value(),
	                                                   json["constraints"]["accel"].number_value()) {
	}

	void calculate() {
		double distance = 0.0;
		this->distance.clear();
		this->t.clear();

		for (size_t i = 0; i <= BEZIER_POINT_DENSITY; i++) {
			double t = static_cast<double>(i) / static_cast<double>(BEZIER_POINT_DENSITY);

			this->t.emplace_back(t);
			this->distance.emplace_back(distance);

			distance += this->getD(t).norm();
		}
	}

	[[nodiscard]] QLength getDistanceAtT(double t) const {
		return interp(this->t, distance, t);
	}

	[[nodiscard]] QLength getDistance() const {
		return this->distance[this->distance.size() - 1];
	}

	[[nodiscard]] Eigen::Vector3d get(const double t) const {
		// TODO!
	}

	[[nodiscard]] Eigen::Vector2d getD(const double t) const {
		// TODO!
	}

	[[nodiscard]] Eigen::Vector2d getDD(const double t) const {
		// TODO!
	}

	[[nodiscard]] QCurvature getCurvature(const double t) const {
		// TODO!
	}

	[[nodiscard]] bool getReversed() const {
		return reversed;
	}

	[[nodiscard]] bool getStopBegin() const {
		return stopBegin;
	}
};
