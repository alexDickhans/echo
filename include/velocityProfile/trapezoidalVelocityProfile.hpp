#pragma once

#include <iostream>
#include "units/units.hpp"

struct ProfileConstraints {
	QVelocity maxVelocity;
	QAcceleration maxAcceleration;
};

class TrapezoidalVelocityProfile {
	QLength distance;

	ProfileConstraints profileConstraints;

	QVelocity initialSpeed = 0.0, endSpeed = 0.0;

	QTime ta, ts, td;
	QVelocity cruiseSpeed;
	QAcceleration aa, ad;
public:
	TrapezoidalVelocityProfile(QLength distance, ProfileConstraints profileConstraints, QVelocity initialSpeed = 0.0, QVelocity endSpeed = 0.0);

	QTime getDuration() const;
	QLength getDistanceByTime(QTime time);
	QVelocity getVelocityByTime(QTime time);
	QAcceleration getAccelerationByTime(QTime time);

	[[nodiscard]] const QVelocity &getInitialSpeed() const;
	void setInitialSpeed(const QVelocity &initialSpeed);
	[[nodiscard]] const QVelocity &getEndSpeed() const;
	void setEndSpeed(const QVelocity &endSpeed);

	[[nodiscard]] const QLength &getDistance() const;
	virtual void setDistance(const QLength &distance);
	[[nodiscard]] const ProfileConstraints &getProfileConstraints() const;
	virtual void setProfileConstraints(const ProfileConstraints &profileConstraints);

	void calculate();

	virtual ~TrapezoidalVelocityProfile() = default;
};