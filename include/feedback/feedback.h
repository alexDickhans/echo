#pragma once

class FeedbackController {
protected:
	double target = 0;
public:
	virtual double update(double input) { return 0.0; }

	void setTarget(const double target) {
		this->target = target;
	}

	[[nodiscard]] double getTarget() const {
		return this->target;
	}

	virtual ~FeedbackController() = default;
};