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

class DrivetrainSubsystem : public Subsystem {
private:
    pros::MotorGroup left11W, right11W;
    pros::Imu imu;
    pros::adi::DigitalOut pto;
    pros::Rotation winchRotation;
    bool ptoActive = false;

    QLength odomChange;
    QLength lastOdom;

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

    Eigen::Vector3f exponentialPose = Eigen::Vector3f::Zero();

    Angle lastTheta = 0.0;

    pros::Rotation odom;

    QLength onPtoActivateStringPosition = 0.0;

public:
    DrivetrainSubsystem(const std::initializer_list<int8_t> &left11_w, const std::initializer_list<int8_t> &right11_w,
                        pros::Imu imu, pros::adi::DigitalOut pto, pros::Rotation winchRotation,
                        std::function<bool()> hasGoal, pros::Rotation odom) : left11W(left11_w), right11W(right11_w), imu(std::move(imu)),
                                                         pto(std::move(pto)), winchRotation(std::move(winchRotation)),
                                                         particleFilter([this, imu]() {
                                                             const Angle angle = -imu.get_rotation() * degree;
                                                             return isfinite(angle.getValue()) ? angle : 0.0;
                                                         }),
                                                         hasGoal(std::move(hasGoal)), odom(std::move(odom)) {
        this->lastOdom = 0.0;

        left11W.set_gearing_all(pros::MotorGears::blue);
        right11W.set_gearing_all(pros::MotorGears::blue);

        left11W.set_encoder_units_all(pros::MotorEncoderUnits::rotations);
        right11W.set_encoder_units_all(pros::MotorEncoderUnits::rotations);

        lastOdom = this->getOdomDistance();

        winchRotation.reset_position();

        imu.reset(pros::competition::is_autonomous() || pros::competition::is_disabled() || AUTON == SKILLS);
    }

    void addLocalizationSensor(Sensor *sensor) { particleFilter.addSensor(sensor); }

    void periodic() override {
        const QLength odomReading = this->getOdomDistance();

        odomChange = odomReading - lastOdom;

        lastOdom = odomReading;

        std::uniform_real_distribution avgDistribution(odomChange.getValue() - CONFIG::DRIVE_NOISE * odomChange.getValue(),
                                                       odomChange.getValue() + CONFIG::DRIVE_NOISE * odomChange.getValue());
        std::uniform_real_distribution angleDistribution(
            particleFilter.getAngle().getValue() - CONFIG::ANGLE_NOISE.getValue(),
            particleFilter.getAngle().getValue() + CONFIG::ANGLE_NOISE.getValue());

        // Exponential Pose Tracking
        const Angle dTheta = particleFilter.getAngle() - lastTheta;

        const auto localMeasurement = Eigen::Vector2f({odomChange.getValue(), 0});
        const auto displacementMatrix =
                Eigen::Matrix2d({
                    {1.0 - pow(dTheta.getValue(), 2), -dTheta.getValue() / 2.0},
                    {dTheta.getValue() / 2.0, 1.0 - pow(dTheta.getValue(), 2)}
                })
                .cast<float>();

        auto time = pros::micros();

        particleFilter.update([this, angleDistribution, avgDistribution, displacementMatrix]() mutable {
            const auto noisy = avgDistribution(de);
            const auto angle = angleDistribution(de);

            return Eigen::Rotation2Df(angle) * Eigen::Vector2f({noisy, 0.0});
        });

        const Eigen::Vector2f localDisplacement = displacementMatrix * localMeasurement;
        const Eigen::Vector2f globalDisplacement =
                Eigen::Rotation2Df(particleFilter.getAngle().Convert(radian)) * localDisplacement;

        exponentialPose += Eigen::Vector3f(globalDisplacement.x(), globalDisplacement.y(), dTheta.Convert(radian));

        lastTheta = particleFilter.getAngle();

        if (recording) {
            uLinear.emplace_back(lastULinear);
            uAngular.emplace_back(lastUAngular);

            auto currentXLinear = (100.0 * odomChange).getValue();
            auto currentXAngular = (-imu.get_gyro_rate().z) * (degree / second).getValue();

            xLinear.emplace_back(currentXLinear);
            xAngular.emplace_back(currentXAngular);
        }
    }

    bool odomConnected() {
        return odom.is_installed();
    }

    Angle getAngle() const { return -imu.get_rotation() * degree; }

    Eigen::Rotation2Dd getRotation() const { return Eigen::Rotation2Dd(this->getAngle().getValue()); }

    void setPct(const double left, const double right) {
        this->left11W.move_voltage(left * 12000.0);
        this->right11W.move_voltage(right * 12000.0);

        lastULinear = (left + right) / 2.0;
        lastUAngular = (right - left) / 2.0;
    }

    void setDriveSpeeds(DriveSpeeds lastSpeeds, DriveSpeeds nextDriveSpeeds = {0.0, 0.0}) {
        const QAngularAcceleration angularAcceleration =
            (nextDriveSpeeds.angularVelocity - lastSpeeds.angularVelocity) / 10_ms;
        const QAcceleration linearAcceleration = (nextDriveSpeeds.linearVelocity - lastSpeeds.linearVelocity) / 10_ms;

        const bool goal = robotHasGoal();

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

    bool robotHasGoal() {
        return hasGoal();
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
        auto stringDistance = (this->right11W.get_position(0) + this->right11W.get_position(1) +
                         this->left11W.get_position(0) + this->left11W.get_position(1)) *
                            M_PI * 0.5 * CONFIG::WINCH_RADIUS +
                        CONFIG::START_STRING_LENGTH - this->onPtoActivateStringPosition;

        return stringDistance;
    }

    QLength getOdomDistance() const {
        auto distance = (this->odom.get_position() / 36000.0) / CONFIG::DRIVE_RATIO * 2.0 * M_PI *
                        CONFIG::DRIVETRAIN_TUNING_SCALAR * CONFIG::DRIVE_RADIUS;

        return distance;
    }

    QLength getDistance() const { return this->getOdomDistance(); }

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
                                      flip]() {
                                      this->initUniform(minX, minY, maxX, maxY, angle, flip);
                                  },
                                  {this});
    }

    InstantCommand *setNorm(const Eigen::Vector2f &mean, const Eigen::Matrix2f &covariance, const Angle &angle,
                            const bool flip) {
        exponentialPose = Eigen::Vector3f(mean.x(), mean.y(), angle.Convert(radian));
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

        pros::delay(100);

        auto redWeight = 0.0;
        auto blueWeight = 0.0;

        for (int i = 0; i < 5; ++i) {
            particleFilter.updateSensors();

            redWeight += particleFilter.weightParticle(redPosition);
            blueWeight += particleFilter.weightParticle(bluePosition);

            pros::delay(20);
        }

        auto pot = pros::adi::AnalogIn('b');

        if (pot.get_value() > 2000) {
            ALLIANCE = RED;
        } else {
            ALLIANCE = BLUE;
        }

        // pros::lcd::set_text(0, (ALLIANCE == RED ? "RED" : "BLUE") + std::to_string(redWeight) + ", " +
        //                                std::to_string(blueWeight));
        std::cout << (ALLIANCE == RED ? "RED " : "BLUE ") + std::to_string(redWeight) + ", " +
                std::to_string(blueWeight) << std::endl;

        auto controller = pros::Controller(CONTROLLER_MASTER);

        controller.set_text(0, 0, (ALLIANCE == RED ? "RED" : "BLUE"));
    }

    void analyzeSysIdData() const {
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

    InstantCommand *activatePto() {
        return new InstantCommand(
            [this]() {
                this->pto.set_value(true);
                this->onPtoActivateStringPosition = this->getStringDistance();
            },
            {});
    }

    InstantCommand *retractPto() {
        return new InstantCommand(
            [this]() {
                this->pto.set_value(false);
            },
            {});
    }

    FunctionalCommand *hangController(pros::Controller &controller) {
        return new FunctionalCommand([this]() {
                                     },
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

    FunctionalCommand *hangOut(double pct, QLength stringLength) {
        return new FunctionalCommand([this, pct]() mutable { this->setPct(pct, pct); std::cout << "Hang out start" << std::endl; }, [this]() {
                                     },
                                     [this](bool _) {
                                         std::cout << "Hang out end" << std::endl;
                                     },
                                     [this, stringLength]() { return this->getStringDistance() > stringLength; },
                                     {this});
    }

    FunctionalCommand *hangIn(double pct, QLength stringLength) {
        return new FunctionalCommand([this, pct]() mutable { this->setPct(-pct, -pct); std::cout << "Hang in start" << std::endl; }, [this]() {
                                     }, [this](bool _) { std::cout << "Hang in done" << std::endl; },
                                     [this, stringLength]() { return this->getStringDistance() < stringLength; },
                                     {this});
    }

    FunctionalCommand *hangOutNoPto(QLength stringLength) {
        return new FunctionalCommand([this]() mutable { this->retractPto(); }, [this]() {
                                     }, [this](bool _) { this->activatePto(); },
                                     [this, stringLength]() { return this->getStringDistance() > stringLength; },
                                     {this});
    }

    FunctionalCommand *hangPctCommand(double pct) {
        return new FunctionalCommand([this, pct]() { this->setPct(pct, pct); }, [this]() mutable {
                                     }, [this](bool _) {
                                     },
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
                analyzeSysIdData();
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
                analyzeSysIdData();
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
            this->pct(0.7, 0.7)->withTimeout(600_ms),
            this->pct(0.2, 0.2)->withTimeout(600_ms),
            this->pct(0.0, 0.0)->withTimeout(300_ms),
            this->pct(-0.7, -0.7)->withTimeout(800_ms),
            this->pct(-0.2, -0.2)->withTimeout(800_ms),
            this->pct(0.0, 0.0)->withTimeout(300_ms),
            this->pct(0.5, 0.5)->withTimeout(500_ms),
            this->pct(0.7, 0.7)->withTimeout(600_ms),
            this->pct(0.2, 0.2)->withTimeout(600_ms),
            this->pct(0.0, 0.0)->withTimeout(300_ms),
            this->pct(-0.7, -0.7)->withTimeout(800_ms),
            this->pct(-0.2, -0.2)->withTimeout(800_ms),
            new InstantCommand(
                [this]() mutable {
                    this->recording = false;
                    analyzeSysIdData();
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
            this->pct(1.0, -1.0)->withTimeout(400_ms),
            this->pct(-0.5, 0.5)->withTimeout(800_ms),
            this->pct(-0.2, 0.2)->withTimeout(800_ms),
            this->pct(1.0, -1.0)->withTimeout(400_ms),
            this->pct(-0.2, 0.2)->withTimeout(300_ms),
            this->pct(0.5, -0.5)->withTimeout(500_ms),
            this->pct(1.0, -1.0)->withTimeout(400_ms),
            this->pct(-0.5, 0.5)->withTimeout(800_ms),
            this->pct(-0.2, 0.2)->withTimeout(800_ms),
            this->pct(1.0, -1.0)->withTimeout(400_ms),
            this->pct(-0.2, 0.2)->withTimeout(300_ms),
            this->pct(0.5, -0.5)->withTimeout(500_ms),
            this->pct(1.0, -1.0)->withTimeout(400_ms),
            this->pct(-0.5, 0.5)->withTimeout(800_ms),
            this->pct(-0.2, 0.2)->withTimeout(800_ms),
            this->pct(1.0, -1.0)->withTimeout(400_ms),
            this->pct(-0.2, 0.2)->withTimeout(300_ms),
            new InstantCommand(
                [this]() mutable {
                    this->recording = false;
                    analyzeSysIdData();
                },
                {}),
        });
    }

    RunCommand *pct(double left, double right) {
        return new RunCommand([this, left, right]() { this->setPct(left, right); }, {this});
    }

    double getTopMotorTemp() const {
        double maxTemp = 0.0;

        for (auto temp: left11W.get_temperature_all()) {
            maxTemp = std::max(maxTemp, temp);
        }

        for (auto temp: right11W.get_temperature_all()) {
            maxTemp = std::max(maxTemp, temp);
        }

        return maxTemp;
    }

    ~DrivetrainSubsystem() override = default;
};
