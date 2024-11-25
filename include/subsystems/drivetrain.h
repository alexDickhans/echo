#pragma once

#include <utility>

#include "Eigen/Eigen"
#include "command/instantCommand.h"
#include "config.h"
#include "localization/particleFilter.h"
#include "subsystems.h"
#include "telemetry/telemetry.h"

#include "localization/gps.h"

#include "auton.h"
#include "autonomous/autons.h"

#include "sysid/oneDofVelocitySystem.h"

struct DriveSpeeds {
    QVelocity linearVelocity = 0.0;
    QAngularVelocity angularVelocity = 0.0;
};

class Drivetrain : public Subsystem {
private:
    pros::MotorGroup left11W, right11W, left5W, right5W;
    pros::Imu imu;
    pros::adi::DigitalOut pto;
    pros::adi::DigitalOut stringRelease;
    bool ptoActive = false;

    QLength leftChange, rightChange;
    QLength lastLeft, lastRight;

    ParticleFilter<CONFIG::NUM_PARTICLES> particleFilter;

    std::ranlux24_base de;

    GpsSensor *sensor;

    double lastULinear = 0.0;
    double lastUAngular = 0.0;

    std::vector<double> uLinear;
    std::vector<double> xLinear;
    std::vector<double> uAngular;
    std::vector<double> xAngular;

    bool recording = false;

    std::function<bool()> hasGoal;

    QLength stringLength = CONFIG::START_STRING_LENGTH;

    QLength lastStringLength = 0.0;

public:
    Drivetrain(const std::initializer_list<int8_t> &left11_w, const std::initializer_list<int8_t> &right11_w,
               const std::initializer_list<int8_t> &left5_w, const std::initializer_list<int8_t> &right5_w,
               pros::Imu imu, pros::adi::DigitalOut pto, pros::adi::DigitalOut stringRelease,
               std::function<bool()> hasGoal) :
        left11W(left11_w), right11W(right11_w), left5W(left5_w), right5W(right5_w), imu(std::move(imu)),
        pto(std::move(pto)), particleFilter([this, imu]() {
            const Angle angle = -imu.get_rotation() * degree;
            return isfinite(angle.getValue()) ? angle : 0.0;
        }),
        hasGoal(std::move(hasGoal)), stringRelease(stringRelease) {
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

        if (ptoActive != true) {
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
        } else {
            const QLength currentLength = this->getStringDistance();

            const auto change = currentLength - lastStringLength;

            this->stringLength += change;

            lastStringLength = currentLength;
        }

        if (recording) {
            uLinear.emplace_back(lastULinear);
            uAngular.emplace_back(lastUAngular);

            xLinear.emplace_back(((leftChange + rightChange) / (2.0 * 0.01)).getValue());
            xAngular.emplace_back((rightChange - leftChange).Convert(metre) /
                                  (0.01 * CONFIG::TRACK_WIDTH.Convert(metre)));
        }
    }

    Angle getAngle() const { return -imu.get_rotation() * degree; }

    Eigen::Rotation2Dd getRotation() const { return Eigen::Rotation2Dd(this->getAngle().getValue()); }

    void setPct(const double left, const double right) {
        this->left11W.move_voltage(left * 12000.0);
        this->right11W.move_voltage(right * 12000.0);
        this->left5W.move_voltage(left * 8000.0); // 5.5W have a lower max voltage then the 11W motors
        this->right5W.move_voltage(right * 8000.0);

        lastULinear = (left + right) / 2.0;
        lastUAngular = (right - left) / 2.0;
    }

    void setDriveSpeeds(DriveSpeeds lastSpeeds, DriveSpeeds nextDriveSpeeds = {0.0, 0.0}) {
        QAngularAcceleration angularAcceleration =
                (nextDriveSpeeds.angularVelocity - lastSpeeds.angularVelocity) / 10_ms;
        QAcceleration linearAcceleration = (nextDriveSpeeds.linearVelocity - lastSpeeds.linearVelocity) / 10_ms;

        bool goal = hasGoal();

        const double uLinear =
                (goal ? CONFIG::DRIVETRAIN_LINEAR_VELOCITY_FF_GOAL : CONFIG::DRIVETRAIN_LINEAR_VELOCITY_FF_NO_GOAL) *
                Eigen::Vector2d(nextDriveSpeeds.linearVelocity.getValue(), linearAcceleration.getValue());
        const double uAngular =
                (goal ? CONFIG::DRIVETRAIN_ANGULAR_VELOCITY_FF_GOAL : CONFIG::DRIVETRAIN_ANGULAR_VELOCITY_FF_NO_GOAL) *
                Eigen::Vector2d(nextDriveSpeeds.angularVelocity.getValue(), angularAcceleration.getValue());

        double left = uLinear - uAngular;
        double right = uLinear + uAngular;

        left += signnum_c(left) * CONFIG::K_s;
        right += signnum_c(right) * CONFIG::K_s;

        this->setPct(left, right);
    }

    QLength getLeftDistance() const {
        return (this->left11W.get_position(0) + this->left11W.get_position(1)) / 2.0 / CONFIG::DRIVE_RATIO * 2.0 *
               M_PI * CONFIG::DRIVETRAIN_TUNING_SCALAR * CONFIG::DRIVE_RADIUS;
    }

    QLength getRightDistance() const {
        return (this->right11W.get_position(0) + this->right11W.get_position(1)) / 2.0 / CONFIG::DRIVE_RATIO * 2.0 *
               M_PI * CONFIG::DRIVETRAIN_TUNING_SCALAR * CONFIG::DRIVE_RADIUS;
    }

    QLength getStringDistance() const {
        return (this->left11W.get_position(0) + this->left11W.get_position(1) + this->right11W.get_position(0) +
                this->right11W.get_position(1)) /
               4.0 / CONFIG::STRING_RATIO * 2.0 * M_PI * CONFIG::WINCH_RADIUS;
    }

    QLength getDistance() const { return (this->getLeftDistance() + this->getRightDistance()) / 2.0; }

    auto setPto(const bool newValue) -> void { this->pto.set_value(newValue); }

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

        if (redWeight >= blueWeight) {
            ALLIANCE = RED;
        } else {
            ALLIANCE = BLUE;
        }
    }

    void printData() {
        OneDofVelocitySystem linear;
        OneDofVelocitySystem angular;

        linear.characterize(xLinear, uLinear);
        angular.characterize(xAngular, uAngular);

        std::cout << "Linear: " << linear.getFF() << std::endl;
        std::cout << "Angular: " << angular.getFF() << std::endl;
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

    InstantCommand *releaseHang() {
        return new InstantCommand(
                [this]() {
                    this->pto.set_value(true);
                    ptoActive = true;
                    this->lastStringLength = this->getStringDistance();
                    this->stringRelease.set_value(true);
                },
                {});
    }

    FunctionalCommand *hangController(pros::Controller &controller) {
        return new FunctionalCommand([this]() {},
                                     [this, controller]() mutable {
                                         this->setPct(controller.get_analog(ANALOG_LEFT_Y) / 127.0,
                                                      controller.get_analog(ANALOG_LEFT_Y) / 127.0);
                                     },
                                     [this](bool _) {
                                         this->pto.set_value(false);
                                         ptoActive = false;
                                     },
                                     []() { return false; }, {this});
    }

    FunctionalCommand *hangUp(double pct, QLength stringLength) {
        return new FunctionalCommand([this, pct]() mutable { this->setPct(pct, pct); }, [this]() {},
                                     [this](bool _) { std::cout << "Hang" << std::endl; },
                                     [this, stringLength]() { return this->stringLength > stringLength; }, {this});
    }

    FunctionalCommand *hangDown(double pct, QLength stringLength) {
        return new FunctionalCommand([this, pct]() mutable { this->setPct(pct, pct); }, [this]() {}, [this](bool _) {},
                                     [this, stringLength]() { return this->stringLength < stringLength; }, {this});
    }

    FunctionalCommand *hangPctCommand(double pct) {
        return new FunctionalCommand([this, pct]() { this->setPct(pct, pct); }, [this]() mutable {}, [this](bool _) {},
                                     []() { return false; }, {this});
    }

    FunctionalCommand *arcadeRecord(pros::Controller &controller) {
        return new FunctionalCommand(
                [this]() mutable {
                    this->recording = true;
                    uAngular.clear();
                    uLinear.clear();
                    xLinear.clear();
                    xAngular.clear();
                },
                [this, controller]() mutable {
                    this->setPct((controller.get_analog(ANALOG_LEFT_Y) + controller.get_analog(ANALOG_RIGHT_X)) / 127.0,
                                 (controller.get_analog(ANALOG_LEFT_Y) - controller.get_analog(ANALOG_RIGHT_X)) /
                                         127.0);
                },
                [this](bool _) mutable {
                    this->recording = false;
                    printData();
                },
                [this]() mutable { return false; }, {this});
    }

    FunctionalCommand *tankRecord(pros::Controller &controller) {
        return new FunctionalCommand(
                [this]() mutable {
                    this->recording = true;
                    uAngular.clear();
                    uLinear.clear();
                    xLinear.clear();
                    xAngular.clear();
                },
                [this, controller]() mutable {
                    this->setPct(controller.get_analog(ANALOG_LEFT_Y) / 127.0,
                                 controller.get_analog(ANALOG_RIGHT_Y) / 127.0);
                },
                [this](bool _) mutable {
                    this->recording = false;
                    printData();
                },
                [this]() mutable { return false; }, {this});
    }

    Sequence *characterizeLinear() {
        return new Sequence({
                new InstantCommand(
                        [this]() mutable {
                            this->recording = true;
                            uAngular.clear();
                            uLinear.clear();
                            xLinear.clear();
                            xAngular.clear();
                        },
                        {}),
                this->pct(0.5, 0.5)->withTimeout(500_ms),
                this->pct(1.0, 1.0)->withTimeout(200_ms),
                this->pct(-0.5, -0.5)->withTimeout(800_ms),
                this->pct(-0.2, -0.2)->withTimeout(300_ms),
                this->pct(1.0, 1.0)->withTimeout(200_ms),
                new InstantCommand(
                        [this]() mutable {
                            this->recording = false;
                            printData();
                        },
                        {}),
        });
    }

    Sequence *characterizeAngular() {
        return new Sequence({
                new InstantCommand(
                        [this]() mutable {
                            this->recording = true;
                            uAngular.clear();
                            uLinear.clear();
                            xLinear.clear();
                            xAngular.clear();
                        },
                        {}),
                this->pct(0.5, -0.5)->withTimeout(500_ms),
                this->pct(1.0, -1.0)->withTimeout(200_ms),
                this->pct(-0.5, 0.5)->withTimeout(800_ms),
                this->pct(-0.2, 0.2)->withTimeout(300_ms),
                this->pct(1.0, -1.0)->withTimeout(200_ms),
                new InstantCommand(
                        [this]() mutable {
                            this->recording = false;
                            printData();
                        },
                        {}),
        });
    }

    RunCommand *pct(double left, double right) {
        return new RunCommand([this, left, right]() { this->setPct(left, right); }, {this});
    }

    ~Drivetrain() override = default;
};
