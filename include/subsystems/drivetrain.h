#pragma once
#include "Eigen/Eigen"

#include "config.h"

class Drivetrain : public Subsystem {
private:
	pros::MotorGroup left11W, right11W, left5W, right5W;
	pros::Imu imu;

	QLength leftChange, rightChange;
	QLength lastLeft, lastRight;
public:
	Drivetrain(const std::initializer_list<int8_t> &left11_w, const std::initializer_list<int8_t> &right11_w, const std::initializer_list<int8_t> &left5_w,
	           const std::initializer_list<int8_t> &right5_w, pros::Imu imu)
		: left11W(left11_w),
		  right11W(right11_w),
		  left5W(left5_w),
		  right5W(right5_w), imu(std::move(imu)) {
		this->leftChange = 0.0;
		this->rightChange = 0.0;

		left11W.set_gearing_all(pros::MotorGears::blue);
		right11W.set_gearing_all(pros::MotorGears::blue);
		left5W.set_gearing_all(pros::MotorGears::green);
		right5W.set_gearing_all(pros::MotorGears::green);

		left11W.set_encoder_units_all(pros::MotorEncoderUnits::rotations);
		right11W.set_encoder_units_all(pros::MotorEncoderUnits::rotations);

		left5W.set_encoder_units_all(pros::MotorEncoderUnits::rotations);
		right5W.set_encoder_units_all(pros::MotorEncoderUnits::rotations);

		imu.reset(true);
	}

	void periodic() override {
		const QLength leftLength = left11W.get_position() * 2.0 * M_PI / CONFIG::DRIVE_RATIO * CONFIG::DRIVE_RADIUS;
		const QLength rightLength = left11W.get_position() * 2.0 * M_PI / CONFIG::DRIVE_RATIO * CONFIG::DRIVE_RADIUS;

		leftChange = leftLength - lastLeft;
		rightChange = rightLength - lastRight;

		lastLeft = leftLength;
		lastRight = rightLength;
	}

	void setPct(double left, double right) {
		this->left11W.move_voltage(left * 12000.0);
		this->right11W.move_voltage(right * 12000.0);
		this->left5W.move_voltage(left * 8000.0); // 5.5W have a lower max voltage then the 11W motors
		this->right5W.move_voltage(right * 8000.0);
	}

	QLength getLeftDistance() const {
		assert(false);
	}

	QLength getRightDistance() const {
		assert(false);
	}

	QLength getDistance() const {
		return (this->getLeftDistance() + this->getRightDistance()) / 2.0;
	}

	void setVelocity(QVelocity left, QVelocity right) {
		assert(false);
	}

	Eigen::Vector3d getPose() const {
		assert(false);
	}

	RunCommand* tank(pros::Controller& controller) {
		return new RunCommand([this, controller]() mutable {
			this->setPct(controller.get_analog(ANALOG_LEFT_Y) / 127.0, controller.get_analog(ANALOG_RIGHT_Y) / 127.0);
		}, {this});
	}

	RunCommand *pct(double left, double right) {
		return new RunCommand([this, left, right] () { this->setPct(left, right); }, {this});
	}

	~Drivetrain() override = default;
};
