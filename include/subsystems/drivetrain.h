#pragma once

#include "Eigen/Eigen"
#include "command/instantCommand.h"
#include "config.h"
#include "localization/particleFilter.h"
#include "subsystems.h"
#include "telemetry/telemetry.h"

#include "localization/gps.h"

#include "auton.h"
#include "autonomous/autons.h"

struct DriveSpeeds {
    QVelocity linearVelocity = 0.0;
    QAngularVelocity angularVelocity = 0.0;
};

class Drivetrain : public Subsystem {
private:
    pros::MotorGroup left11W, right11W, left5W, right5W;
    pros::Imu imu;
    pros::ADIDigitalOut pto;

    QLength leftChange, rightChange;
    QLength lastLeft, lastRight;

    ParticleFilter<CONFIG::NUM_PARTICLES> particleFilter;

    std::ranlux24_base de;

    GpsSensor *sensor;

public:
    Drivetrain(const std::initializer_list<int8_t> &left11_w, const std::initializer_list<int8_t> &right11_w,
               const std::initializer_list<int8_t> &left5_w, const std::initializer_list<int8_t> &right5_w,
               pros::Imu imu, pros::ADIDigitalOut pto) :
        left11W(left11_w), right11W(right11_w), left5W(left5_w), right5W(right5_w), imu(std::move(imu)), pto(pto),
        particleFilter([this, imu]() {
            const Angle angle = -imu.get_rotation() * degree;
            return isfinite(angle.getValue()) ? angle : 0.0;
        }) {
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

        imu.reset(pros::competition::is_autonomous() || pros::competition::is_disabled() || AUTON == SKILLS);
    }

    void addLocalizationSensor(Sensor *sensor) { particleFilter.addSensor(sensor); }

    void periodic() override {
        const QLength leftLength = this->getLeftDistance();
        const QLength rightLength = this->getRightDistance();

        leftChange = leftLength - lastLeft;
        rightChange = rightLength - lastRight;

        lastLeft = leftLength;
        lastRight = rightLength;

        auto avg = (leftChange + rightChange) / 2.0;

        std::uniform_real_distribution avgDistribution(avg.getValue() - CONFIG::DRIVE_NOISE * avg.getValue(),
                                                       avg.getValue() + CONFIG::DRIVE_NOISE * avg.getValue());
        std::uniform_real_distribution angleDistribution(
                particleFilter.getAngle().getValue() - CONFIG::ANGLE_NOISE.getValue(),
                particleFilter.getAngle().getValue() + CONFIG::ANGLE_NOISE.getValue());

        particleFilter.update([this, angleDistribution, avgDistribution]() mutable {
            const auto noisy = avgDistribution(de);
            const auto angle = angleDistribution(de);

            return Eigen::Rotation2Df(angle) * Eigen::Vector2f({noisy, 0.0});
        });
    }

    Angle getAngle() const { return -imu.get_rotation() * degree; }

    Eigen::Rotation2Dd getRotation() const { return Eigen::Rotation2Dd(this->getAngle().getValue()); }

    void setPct(const double left, const double right) {
        this->left11W.move_voltage(left * 12000.0);
        this->right11W.move_voltage(right * 12000.0);
        this->left5W.move_voltage(left * 8000.0); // 5.5W have a lower max voltage then the 11W motors
        this->right5W.move_voltage(right * 8000.0);
    }

    void setDriveSpeeds(DriveSpeeds speeds, DriveSpeeds lastDriveSpeeds = {0.0, 0.0}) {
        const auto angular_wheel_velocity_commanded =
                speeds.angularVelocity.getValue() * CONFIG::TRACK_WIDTH / 2.0 / second;

        QVelocity currentLeft = speeds.linearVelocity - angular_wheel_velocity_commanded;
        QVelocity currentRight = speeds.linearVelocity + angular_wheel_velocity_commanded;

        const auto last_angular_wheel_velocity_commanded =
                speeds.angularVelocity.getValue() * CONFIG::TRACK_WIDTH / 2.0 / second;

        QVelocity lastLeft = lastDriveSpeeds.linearVelocity - last_angular_wheel_velocity_commanded;
        QVelocity lastRight = lastDriveSpeeds.linearVelocity + last_angular_wheel_velocity_commanded;

        QAcceleration accelLeft = (currentLeft - lastLeft) / 10_ms;
        QAcceleration accelRight = (currentRight - lastRight) / 10_ms;

        this->setPct(CONFIG::DRIVETRAIN_FEEDFORWARD(currentLeft, accelLeft),
                           CONFIG::DRIVETRAIN_FEEDFORWARD(currentRight, accelRight));
    }

    QLength getLeftDistance() const {
        return (this->left11W.get_position(0) + this->left11W.get_position(1)) / 2.0 / CONFIG::DRIVE_RATIO * 2.0 *
               M_PI * CONFIG::DRIVETRAIN_TUNING_SCALAR * CONFIG::DRIVE_RADIUS;
    }

    QLength getRightDistance() const {
        return (this->right11W.get_position(0) + this->right11W.get_position(1)) / 2.0 / CONFIG::DRIVE_RATIO * 2.0 *
               M_PI * CONFIG::DRIVETRAIN_TUNING_SCALAR * CONFIG::DRIVE_RADIUS;
    }

    QLength getDistance() const { return (this->getLeftDistance() + this->getRightDistance()) / 2.0; }

    auto setPto(const bool newValue) -> void {
        this->pto.set_value(newValue);
    }

    void setVelocity(const QVelocity left, const QVelocity right) {
        // std::cout << "Left: " << left.Convert(inch/second) << " Right: " << right.Convert(inch/second) << std::endl;

        this->left11W.move_velocity((left / CONFIG::MAX_SPEED).getValue() * 600.0);
        this->right11W.move_velocity((right / CONFIG::MAX_SPEED).getValue() * 600.0);
        this->left5W.move_velocity((left / CONFIG::MAX_SPEED).getValue() * 200.0);
        this->right5W.move_velocity((right / CONFIG::MAX_SPEED).getValue() * 200.0);
    }

    void initNorm(const Eigen::Vector2f &mean, const Eigen::Matrix2f &covariance, const Angle &angle, const bool flip) {
        imu.set_rotation(angle.Convert(degree) * (flip ? 1 : -1));
        this->particleFilter.initNormal(mean, covariance, flip);
    }

    void initUniform(QLength minX, const QLength minY, QLength maxX, QLength maxY, Angle angle, bool flip) {
        imu.set_rotation(angle.Convert(degree) * (flip ? 1 : -1));
        this->particleFilter.initUniform(minX, minY, maxX, maxY);
    }

    Angle getRoll() const { return imu.get_roll() * 1_deg; }

    InstantCommand *setUniform(QLength minX, QLength minY, QLength maxX, QLength maxY, Angle angle, bool flip) {
        return new InstantCommand([this, minX, minY, maxX, maxY, angle,
                                   flip]() { this->initUniform(minX, minY, maxX, maxY, angle, flip); },
                                  {this});
    }

    InstantCommand *setNorm(const Eigen::Vector2f &mean, const Eigen::Matrix2f &covariance, const Angle &angle,
                            const bool flip) {
        return new InstantCommand(
                [this, mean, covariance, flip, angle]() { this->initNorm(mean, covariance, angle, flip); }, {this});
    }

    Eigen::Vector3f getPose() { return this->particleFilter.getPrediction(); }

    std::array<Eigen::Vector3f, CONFIG::NUM_PARTICLES> getParticles() {
        return std::move(particleFilter.getParticles());
    }

    Eigen::Vector3f getParticle(const size_t i) { return std::move(particleFilter.getParticle(i)); }

    void updateAllianceColor(Eigen::Vector3f redPosition) {
        auto bluePosition = Eigen::Vector3f(redPosition.x(), -redPosition.y(), -redPosition.z());

        particleFilter.updateSensors();

        auto redWeight = particleFilter.weightParticle(redPosition);
        auto blueWeight = particleFilter.weightParticle(bluePosition);

        std::cout << redWeight << " " << blueWeight << std::endl;

        if (redWeight >= blueWeight) {
            ALLIANCE = RED;
        } else {
            ALLIANCE = BLUE;
        }
    }

    RunCommand *tank(pros::Controller &controller) {
        return new RunCommand(
                [this, controller]() mutable {
                    this->setPct(controller.get_analog(ANALOG_LEFT_Y) / 127.0,
                                 controller.get_analog(ANALOG_RIGHT_Y) / 127.0);
                },
                {this});
    }

    RunCommand *arcade(pros::Controller &controller) {
        return new RunCommand(
                [this, controller]() mutable {
                    this->setPct((controller.get_analog(ANALOG_LEFT_Y) + controller.get_analog(ANALOG_RIGHT_X)) / 127.0,
                                 (controller.get_analog(ANALOG_LEFT_Y) - controller.get_analog(ANALOG_RIGHT_X)) /
                                         127.0);
                },
                {this});
    }

    RunCommand *pct(double left, double right) {
        return new RunCommand([this, left, right]() { this->setPct(left, right); }, {this});
    }

    RunCommand *velocityCommand(const QVelocity left, const QVelocity right) {
        return new RunCommand([this, left, right]() { this->setVelocity(left, right); }, {this});
    }

    ~Drivetrain() override = default;
};
