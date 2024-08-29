#pragma once

#include "Eigen/Eigen"

struct MotionCommand {
	Eigen::Vector3d desiredPose;
	QVelocity desiredVelocity;
	QAngularVelocity desiredAngularVelocity;
};

class MotionProfile {
public:
	virtual std::optional<MotionCommand> get(QTime t) = 0;

	virtual QTime getDuration() = 0;

	virtual ~MotionProfile() = default;
};