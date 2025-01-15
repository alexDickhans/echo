#include "utils/utils.h"

double normal_pdf(const double x, const double mu, const double sigma) {
    const auto exponent = -pow(x - mu, 2) / (2.0 * pow(sigma, 2));
    return (1.0 / (sigma * sqrt(2.0 * M_PI))) * pow(M_E, exponent);
}

float cheap_norm_pdf(const float x) {// Approximation of the standard normal PDF
    // Coefficients for the rational approximation
    const float a = 0.3989422804014337; // 1 / sqrt(2 * PI)
    const float e = 0.59422804014337;

    // Compute the approximate normal PDF using a rational polynomial
    const float pdfApprox = a / (1.0 + e * x * x * x * x);

    return pdfApprox;
}

float sinc(const Angle angle) {
    if (angle.getValue() == 0.0) {
        return 1.0;
    }

    return sin(angle) / angle.getValue();
}

double interp(const std::vector<double>& x, const std::vector<double>& y, const double x0) {
    const auto min_len = std::min(x.size(), y.size());

    if (min_len == 0) {
        return 0.0;
    }

    if (min_len == 1) {
        return y[0];
    }

    for (size_t i = 1; i < min_len; i ++) {
        if (x0 < x[i] || i == min_len - 1) {
            const auto dx = x[i] - x[i-1];
            const auto dy = y[i] - y[i-1];

            return y[i-1] + (x0 - x[i-1]) * (dy/dx);
        }
    }

    return 0.0;
}

double signnum_c(double x) {
    if (x > 0.0) {
        return 1.0;
    } else if (x < 0.0) {
        return -1.0;
    }
    return 0.0;
}

Angle angleDifference(const Angle x, const Angle y) {
    return std::remainder(x.Convert(radian) - y.Convert(radian), M_TWOPI);
}