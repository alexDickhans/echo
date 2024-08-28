#pragma once
#include "feedback.h"
#include <cmath>


class PID final : public FeedbackController {
private:
		double kP;
		double kI;
		double kD;

		double error;
		double totalError = 0.0;
		double prevError = 0.0;
		double derivitive;

		double integralBound = 3000.0;
		double maxIntegral = 30.0;

		double power;

		// bool turnPid = false;

	protected:
		double calculatePidValues(double input) {

			this->error = target - input;

			// if (turnPid) {
			// 	this->error = angleDifference(target, input);
			// }

			this->derivitive = this->error - this->prevError;

			if (abs(error) < integralBound) {
				totalError += error;
			}
			else {
				totalError = 0;
			}

			totalError = abs(totalError) > maxIntegral ? copysign(1.0, totalError) * maxIntegral : totalError;

			this->power = (error * kP) + (derivitive * kD) + (totalError * kI);

			this->prevError = this->error;

			return this->power;
		}

public:
	PID();
	PID(double kP, double kI, double kD, double target = 0, double position = 0, bool turnPid = false);

	double update(double input);

	void operator=(PID pid) {
		this->kP = pid.getKP();
		this->kI = pid.getKI();
		this->kD = pid.getKD();
		this->integralBound = pid.getIntegralBound();
		this->maxIntegral = pid.getMaxIntegral();
	}

	void operator=(PID* pid) {
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
		this->prevError = 0;
		this->error = 0;
		this->derivitive = 0;
	}

	~PID() override;
};