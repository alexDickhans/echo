#pragma once

#include "units/units.hpp"
#include <vector>

double normal_pdf(double x, double mu, double sigma);

float cheap_norm_pdf(const float x);

Angle angleDifference(const Angle x, const Angle y);

float sinc(const Angle angle);

double interp(const std::vector<double>& x, const std::vector<double>& y, const double x0);

double signnum_c(double x);
