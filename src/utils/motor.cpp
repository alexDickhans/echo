#include "utils/motor.h"

float MOTOR11W::torquePercent(float pct) {
    // Motor torque limits bring the max torque to a constant 100% at and below around 58% speed
    if (pct < 0.58) {
        return 1.0f;
    }

    // Use a linear model for the remaining 42% of the speed range, where the torque falls off to zero just like a
    // regular DC motor VEX messing it up
    return -1.59524f * pct + 1.92143f;
}
