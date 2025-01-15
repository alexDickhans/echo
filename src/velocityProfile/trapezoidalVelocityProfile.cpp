
#include "velocityProfile/trapezoidalVelocityProfile.hpp"

TrapezoidalVelocityProfile::TrapezoidalVelocityProfile(QLength distance, ProfileConstraints profileConstraints, QVelocity initialSpeed, QVelocity endSpeed) : distance(distance), profileConstraints(profileConstraints), initialSpeed(initialSpeed), endSpeed(endSpeed) {
}

QTime TrapezoidalVelocityProfile::getDuration() const {
	return td;
}

QLength TrapezoidalVelocityProfile::getDistanceByTime(const QTime time) {
	if (time <= ta) {
		return 0.5 * Qsq(time) * aa + this->getInitialSpeed() * time;
	} else if (time <= ts) {
		return 0.5 * Qsq(ta) * aa + this->getInitialSpeed() * ta + cruiseSpeed * (time - ta);
	} else if (time <= td) {
		return getDistanceByTime(ts) + 0.5 * Qsq(time - ts) * ad + (time - ts) * cruiseSpeed;
	}

	return this->getDistance();
}


QVelocity TrapezoidalVelocityProfile::getVelocityByTime(QTime time) {
	if (time <= ta) {
		return time * aa + this->getInitialSpeed();
	} else if (time <= ts) {
		return cruiseSpeed;
	} else if (time <= td) {
		return (time - ts) * ad + cruiseSpeed;
	}
	return 0.0;
}

QAcceleration TrapezoidalVelocityProfile::getAccelerationByTime(QTime time) {
	if (time <= ta) {
		return aa;
	} else if (time <= ts) {
		return 0.0;
	} else if (time <= td) {
		return ad;
	}
	return 0.0;
}

void TrapezoidalVelocityProfile::calculate() {
	ProfileConstraints profileConstraints = this->getProfileConstraints();
	cruiseSpeed = copysign(1.0, this->getDistance().getValue()) * profileConstraints.maxVelocity;
	aa = copysign(1.0, (cruiseSpeed - this->getInitialSpeed()).getValue()) * profileConstraints.maxAcceleration;
	ad = copysign(1.0, (this->getEndSpeed() - cruiseSpeed).getValue()) * profileConstraints.maxAcceleration;
	ta = aa.getValue() == 0.0 ? 0.0 : (cruiseSpeed - this->getInitialSpeed()) / aa;
	td = ad.getValue() == 0.0 ? 0.0 : (this->getEndSpeed() - cruiseSpeed) / ad;
	QLength la = 0.5 * Qsq(ta) * aa + this->getInitialSpeed() * ta;
	QLength ld = 0.5 * Qsq(td) * ad + cruiseSpeed * td;

	QLength ls = this->getDistance() - la - ld;

	if (ls.getValue() * copysign(1.0, cruiseSpeed.getValue()) > 0) {
		ts = ls / cruiseSpeed + ta;
		td = ts + td;
	} else {
		QAcceleration aa2 = profileConstraints.maxAcceleration;
		ta = Qabs((this->getEndSpeed() - this->getInitialSpeed())) / aa2;
		la = 0.5 * Qsq(ta) * aa2 + this->getInitialSpeed() * ta;
		if (abs(la.getValue()) > abs(this->getDistance().getValue())) {
			ta = 2 * this->getDistance() / (this->getInitialSpeed() + this->getEndSpeed());
			aa = (this->getEndSpeed() - this->getInitialSpeed())/ta;
			ts = ta;
			td = ta;
		} else {
			ta = (sqrt(2.0) *
					Qsqrt(2 * Qabs(aa) * Qabs(this->getDistance()) + Qsq(this->getInitialSpeed()) + Qsq(this->getEndSpeed())) - 2 * this->getInitialSpeed())/
					(2 * Qabs(aa));
			ts = ta;
			td = 2.0 * ta + ((this->getInitialSpeed() - this->getEndSpeed())/aa);
			cruiseSpeed = this->getInitialSpeed() + ta * aa;
		}
	}
}

const QVelocity &TrapezoidalVelocityProfile::getInitialSpeed() const {
	return initialSpeed;
}
void TrapezoidalVelocityProfile::setInitialSpeed(const QVelocity &initialSpeed) {
	this->initialSpeed = initialSpeed;
}
const QVelocity &TrapezoidalVelocityProfile::getEndSpeed() const {
	return endSpeed;
}
void TrapezoidalVelocityProfile::setEndSpeed(const QVelocity &endSpeed) {
	this->endSpeed = endSpeed;
}
const QLength &TrapezoidalVelocityProfile::getDistance() const {
	return distance;
}
void TrapezoidalVelocityProfile::setDistance(const QLength &distance) {
	this->distance = distance;
}
const ProfileConstraints &TrapezoidalVelocityProfile::getProfileConstraints() const {
	return profileConstraints;
}
void TrapezoidalVelocityProfile::setProfileConstraints(const ProfileConstraints &profileConstraints) {
	this->profileConstraints = profileConstraints;
}
