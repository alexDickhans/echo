#pragma once

#include "units/units.hpp"

class MOTOR11W {
public:
    static float torquePercent(float pct) {
        if (pct < 0.58) {
            return 1.0f;
        }
        return -1.59524f * pct + 1.92143f;
    }
};
