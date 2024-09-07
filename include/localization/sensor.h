#pragma once

class Sensor {
public:
	virtual std::optional<double> p(const Eigen::Vector3f& x) = 0;
	virtual void update() = 0;
	virtual ~Sensor() = default;
};