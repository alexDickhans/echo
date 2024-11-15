#pragma once

#include "command/command.h"
#include "motionProfiling/motionProfile.h"
#include "subsystems/drivetrain.h"
#include "units/units.hpp"
#include "utils/utils.h"

/**
 * Class to follow motion profiled paths
 */
class Ramsete : public Command {
private:
    Drivetrain *drivetrain;

    float zeta;
    float beta;

    QTime startTime;

    MotionProfile *motionProfile;

    DriveSpeeds lastSpeeds;

public:
    Ramsete(Drivetrain *drivetrain, MotionProfile *motion_profile, const float zeta = CONFIG::RAMSETE_ZETA,
            const float beta = CONFIG::RAMSETE_BETA) :
        drivetrain(drivetrain), zeta(zeta), beta(beta), motionProfile(motion_profile) {
        startTime = 0.0;
    }

    void initialize() override { startTime = pros::millis() * millisecond; }

    void execute() override {
        if (const auto command = motionProfile->get(pros::millis() * millisecond - startTime); command.has_value()) {
            Eigen::Vector3f currentPose = drivetrain->getPose();
            Eigen::Vector3f desiredPose = command->desiredPose.cast<float>();

            Eigen::Vector2f error = Eigen::Rotation2Df(-currentPose.z()) * (desiredPose - currentPose).head<2>();

            const Angle errorAngle = angleDifference(desiredPose.z(), currentPose.z()).getValue();

            const auto k = 2.0f * this->zeta *
                           sqrt(Qsq(command->desiredAngularVelocity).getValue() +
                                this->beta * Qsq(command->desiredVelocity).getValue());

            const auto velocity_commanded = cos(errorAngle) * command->desiredVelocity + k * error.x() * metre / second;
            const auto angular_wheel_velocity_commanded =
                    (command->desiredAngularVelocity.getValue() + k * errorAngle.getValue() +
                     this->beta * command->desiredVelocity.getValue() * sinc(errorAngle) * error.y());

            drivetrain->setDriveSpeeds(lastSpeeds, {velocity_commanded, angular_wheel_velocity_commanded.getValue()});
            lastSpeeds = {velocity_commanded, angular_wheel_velocity_commanded};
        }
    }

    void end(bool interrupted) override { std::cout << "DONE" << std::endl; }

    bool isFinished() override { return motionProfile->getDuration() < pros::millis() * millisecond - startTime; }

    std::vector<Subsystem *> getRequirements() override { return {drivetrain}; }
};
