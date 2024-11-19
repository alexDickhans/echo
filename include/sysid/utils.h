#pragma once

double signnum(double x) {
    if (x > 0) return 1;
    if (x < 0) return -1;
    return 0;
}