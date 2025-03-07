#pragma once
#include "feedback.h"
#include <cmath>
#include "utils/utils.h"


class PID final : public FeedbackController {
private:
    double kP;
    double kI;
    double kD;

    double error;
    double totalError = 0.0;
    double prevInput = 0.0;
    double derivitive;

    double integralBound = 3000.0;
    double maxIntegral = 30.0;

    double power;

    bool turnPid = false;

protected:
    double calculatePidValues(double input) {
        this->error = target - input;

        if (turnPid) {
            this->error = angleDifference(target, input).getValue();
        }

        this->derivitive = -(input - this->prevInput);

        if (abs(error) < integralBound) {
            totalError += error;
        } else {
            totalError = 0;
        }

        totalError = abs(totalError) > maxIntegral ? copysign(1.0, totalError) * maxIntegral : totalError;

        this->power = (error * kP) + (derivitive * kD) + (totalError * kI);

        this->prevInput = input;

        return this->power;
    }

public:
    PID() {
        this->kP = 0.0;
        this->kI = 0.0;
        this->kD = 0.0;
        this->target = 0.0;
    }

    PID(const double kP, const double kI, const double kD, const double target = 0, const bool turnPid = false) {
        this->kP = kP;
        this->kI = kI;
        this->kD = kD;
        this->target = target;
        this->turnPid = turnPid;
    }

    PID(const double k_p, const double k_i, const double k_d, const double integral_bound,
        const double max_integral)
        : kP(k_p),
          kI(k_i),
          kD(k_d), error(0), derivitive(0),
          integralBound(integral_bound),
          maxIntegral(max_integral), power(0) {
    }

    double update(double input) {
        return calculatePidValues(input);
    }

    void operator=(PID pid) {
        this->kP = pid.getKP();
        this->kI = pid.getKI();
        this->kD = pid.getKD();
        this->integralBound = pid.getIntegralBound();
        this->maxIntegral = pid.getMaxIntegral();
    }

    void operator=(PID *pid) {
        this->kP = pid->getKP();
        this->kI = pid->getKI();
        this->kD = pid->getKD();
        this->integralBound = pid->getIntegralBound();
        this->maxIntegral = pid->getMaxIntegral();
    }

    double getDerivitive() {
        return derivitive;
    }

    double getError() {
        return error;
    }

    double getKP() {
        return kP;
    }

    void setKP(double kP) {
        this->kP = kP;
    }

    double getKI() {
        return kI;
    }

    void setKI(double kI) {
        this->kI = kI;
    }

    double getKD() {
        return kD;
    }

    void setKD(double kD) {
        this->kD = kD;
    }

    double getIntegralBound() {
        return this->integralBound;
    }

    void setIntegralBound(double integralBound) {
        this->integralBound = integralBound;
    }

    double getMaxIntegral() {
        return this->maxIntegral;
    }

    void setMaxIntegral(double maxIntegral) {
        this->maxIntegral = maxIntegral;
    }

    void reset() {
        this->prevInput = 0;
        this->error = 0;
        this->derivitive = 0;
    }

    [[nodiscard]] bool getTurnPid() const {
        return turnPid;
    }

    void setTurnPid(const bool turn_pid) {
        turnPid = turn_pid;
    }

    ~PID() override = default;
};
