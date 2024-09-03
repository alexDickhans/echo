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

inline double interp(const std::vector<double>& x, const std::vector<double>& y, const double x0) {
	const auto min_len = std::min(x.size(), y.size());

	if (min_len == 0) {
		return 0.0;
	}

	if (min_len == 1) {
		return y[0];
	}

	for (size_t i = 1; i < min_len; i ++) {
		if (x[i] > x0 || i == min_len - 1) {
			const auto dx = x[i] - x[i-1];
			const auto dy = y[i] - y[i-1];

			return y[i-1] + dy * ((x0-x[i-1]) / dx);
		}
	}

	return 0.0;
}
