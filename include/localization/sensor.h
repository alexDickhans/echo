#pragma once

class Sensor {
public:
	virtual std::optional<double> p(Eigen::Vector3d x) = 0;
	virtual void update() = 0;
	virtual ~Sensor() = default;
};