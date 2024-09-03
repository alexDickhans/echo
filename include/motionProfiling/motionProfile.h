#pragma once

#include "Eigen/Eigen"
#include "units/units.hpp"

struct MotionCommand {
	Eigen::Vector3d desiredPose;
	QVelocity desiredVelocity;
	QAngularVelocity desiredAngularVelocity;
	double desiredT;
};

class MotionProfile {
public:
	virtual std::optional<MotionCommand> get(QTime t) = 0;

	[[nodiscard]] virtual QTime getDuration() const = 0;

	virtual double maxT() = 0;

	virtual ~MotionProfile() = default;
};