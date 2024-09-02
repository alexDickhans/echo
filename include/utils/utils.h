#pragma once

inline Angle angleDifference(const Angle x, const Angle y) {
	return fmod((x.Convert(radian) - y.Convert(radian) + M_PI), M_TWOPI) - M_PI;
}

double sinc(Angle angle) {
	if (angle.getValue() == 0.0) {
		return 1.0;
	}

	return sin(angle) / angle.getValue();
}

double interp(std::vector<double> x, std::vector<double> y, double x0) {
	// TODO

	return 0.0;
}
