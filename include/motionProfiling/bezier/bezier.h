#pragma once

#include "json/json.hpp"
#include "utils/utils.h"

#include "Eigen/Eigen"

constexpr size_t BEZIER_POINT_DENSITY = 50;


inline auto BEZIER_MATRIX = Eigen::Matrix4d(
	{{-1.0, 3.0, -3.0, 1.0}, {3.0, -6.0, 3.0, 0.0}, {-3.0, 3.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}}
);

inline auto BEZIER_D_MATRIX = Eigen::Matrix<double, 3, 4>(
	{{-3.0, 9.0, -9.0, 3.0}, {6.0, -12.0, 6.0, 0.0}, {-3.0, 3.0, 0.0, 0.0}}
);

inline auto BEZIER_DD_MATRIX =
		Eigen::Matrix<double, 2, 4>({{-6.0, 18.0, -18.0, 6.0}, {6.0, -12.0, 6.0, 0.0}});

class Bezier {
private:
	Eigen::Vector2d a, b, c, d;
	bool reversed{};
	bool stopBegin{};
	std::vector<double> distance, t;

	static Eigen::Vector2d toVector(const json11::Json &json, bool mirror) {
		return {json["x"].number_value(), json["y"].number_value() * (mirror ? -1.0 : 1.0)};
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

	explicit Bezier(const json11::Json &json, bool mirror) : Bezier(toVector(json["path"][0], mirror), toVector(json["path"][1], mirror),
	                                                   toVector(json["path"][2], mirror), toVector(json["path"][3], mirror),
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

			distance += this->getD(t).norm() / static_cast<double>(BEZIER_POINT_DENSITY);
		}
	}

	[[nodiscard]] QLength getDistanceAtT(const double t) const {
		return interp(this->t, distance, t);
	}

	[[nodiscard]] QLength getDistance() const {
		return this->distance[this->distance.size() - 1];
	}

	[[nodiscard]] Eigen::Vector3d get(const double t) const {
		const auto tVector = Eigen::RowVector4d(pow(t, 3), pow(t, 2), t, 1.0);
		const double x = tVector * BEZIER_MATRIX * Eigen::Vector4d(a.x(), b.x(), c.x(), d.x());
		const double y = tVector * BEZIER_MATRIX * Eigen::Vector4d(a.y(), b.y(), c.y(), d.y());

		const auto d = this->getD(t);

		return {x, y, atan2(d.y(), d.x())};
	}

	[[nodiscard]] Eigen::Vector2d getD(const double t) const {
		const auto tVector = Eigen::RowVector3d(pow(t, 2), t, 1.0);
		const double x = tVector * BEZIER_D_MATRIX * Eigen::Vector4d(a.x(), b.x(), c.x(), d.x());
		const double y = tVector * BEZIER_D_MATRIX * Eigen::Vector4d(a.y(), b.y(), c.y(), d.y());

		return {x, y};
	}

	[[nodiscard]] Eigen::Vector2d getDD(const double t) const {
		const auto tVector = Eigen::RowVector2d(t, 1.0);
		const double x = tVector * BEZIER_DD_MATRIX * Eigen::Vector4d(a.x(), b.x(), c.x(), d.x());
		const double y = tVector * BEZIER_DD_MATRIX * Eigen::Vector4d(a.y(), b.y(), c.y(), d.y());

		return {x, y};
	}

	[[nodiscard]] QCurvature getCurvature(const double tIn) const {
		const double t = std::clamp(tIn, 0.0, 1.0);

		auto D = this->getD(t);
		auto DD = this->getDD(t);

		return (D.x() * DD.y() - DD.x() * D.y()) / pow(this->getD(t).norm(), 3);
	}

	[[nodiscard]] bool getReversed() const {
		return reversed;
	}

	[[nodiscard]] bool getStopBegin() const {
		return stopBegin;
	}
};
