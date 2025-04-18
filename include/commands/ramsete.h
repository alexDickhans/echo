#pragma once

#include "command/command.h"
#include "motionProfiling/motionProfile.h"
#include "subsystems/drivetrain.h"
#include "units/units.hpp"
#include "utils/utils.h"
#include "telemetry/telemetry.h"

/**
 * Class to follow motion profiled paths
 */
class Ramsete : public Command {
private:
    DrivetrainSubsystem *drivetrain;

    float zeta;
    float beta;

    QTime startTime;

    MotionProfile *motionProfile;

    QVelocity lastLeft = 0.0, lastRight = 0.0;

    DriveSpeeds lastSpeeds{0.0, 0.0};

public:
    Ramsete(DrivetrainSubsystem *drivetrain, MotionProfile *motion_profile, const float zeta = CONFIG::RAMSETE_ZETA,
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

            Angle errorAngle = angleDifference(desiredPose.z(), currentPose.z()).getValue();

            const auto k = 2.0f * this->zeta *
                           sqrt(Qsq(command->desiredAngularVelocity).getValue() +
                                this->beta * Qsq(command->desiredVelocity).getValue());

            const auto velocity_commanded = cos(errorAngle) * command->desiredVelocity + k * error.x() * metre / second;
            const auto angular_wheel_velocity_commanded =
                    (command->desiredAngularVelocity.getValue() + k * errorAngle.getValue() +
                     this->beta * command->desiredVelocity.getValue() * sinc(errorAngle) * error.y());

            drivetrain->setDriveSpeeds(lastSpeeds,
                                       {velocity_commanded, angular_wheel_velocity_commanded * radian / second});
            lastSpeeds = {velocity_commanded, angular_wheel_velocity_commanded};

            TELEMETRY.send("{\"time\": " + std::to_string(pros::millis() / 1000.0) + ", \"data\":[");
            TELEMETRY.send("[");
            TELEMETRY.send(std::to_string(currentPose.x()));
            TELEMETRY.send(",");
            TELEMETRY.send(std::to_string(currentPose.y()));
            TELEMETRY.send(",");
            TELEMETRY.send(std::to_string(currentPose.z()));
            TELEMETRY.send("],");
            TELEMETRY.send("[");
            TELEMETRY.send(std::to_string(desiredPose.x()));
            TELEMETRY.send(",");
            TELEMETRY.send(std::to_string(desiredPose.y()));
            TELEMETRY.send(",");
            TELEMETRY.send(std::to_string(desiredPose.z()));
            TELEMETRY.send("]]}\n");
        }
    }

    void end(bool interrupted) override { std::cout << "DONE" << std::endl; }

    bool isFinished() override { return motionProfile->getDuration() < pros::millis() * millisecond - startTime; }

    std::vector<Subsystem *> getRequirements() override { return {drivetrain}; }
};