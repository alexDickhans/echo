#include "utils/motor.h"

float MOTOR11W::torquePercent(float pct) {
    if (pct < 0.58) {
        return 1.0f;
    }
    return -1.59524f * pct + 1.92143f;
}