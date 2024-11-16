#pragma once

#include "command/command.h"
#include "motionProfiling/motionProfile.h"
#include "subsystems/drivetrain.h"
#include "units/units.hpp"
#include "utils/linear.h"
#include "utils/utils.h"

/**
 * Class to follow motion profiled paths
 */
class LTVUnicycleController : public Command {
private:
    Drivetrain *drivetrain;

    Eigen::Matrix3f Q;
    Eigen::Matrix2f R;

    QTime startTime;

    MotionProfile *motionProfile;

    DriveSpeeds lastSpeeds;

public:
    LTVUnicycleController(Drivetrain *drivetrain, MotionProfile *motion_profile,
                          Eigen::Matrix3f Q = CONFIG::DEFAULT_DT_COST_Q,
                          Eigen::Matrix2f R = CONFIG::DEFAULT_DT_COST_R) :
        drivetrain(drivetrain), Q(Q), R(R), motionProfile(motion_profile) {
        startTime = 0.0;
    }

    void initialize() override { startTime = pros::millis() * millisecond; }

    void execute() override {
        if (const auto command = motionProfile->get(pros::millis() * millisecond - startTime); command.has_value()) {
            const Eigen::Vector3f currentPose = drivetrain->getPose();
            const Eigen::Vector3f desiredPose = command->desiredPose.cast<float>();

            const Eigen::Vector2f errorXY = Eigen::Rotation2Df(-currentPose.z()) * (desiredPose - currentPose).head<2>();

            const Angle errorAngle = angleDifference(desiredPose.z(), currentPose.z()).getValue();

            const Eigen::Vector3f error(errorXY.x(), errorXY.y(), errorAngle.Convert(radian));

            const Eigen::Matrix3f A{
                    {0.0, 0.0, 0.0}, {0.0, 0.0, command.value().desiredVelocity.value}, {0.0, 0.0, 0.0}};
            const Eigen::Matrix<float, 3, 2> B{{1.0, 0.0}, {0.0, 0.0}, {0.0, 1.0}};

            const auto discAB = discretizeAB(A, B, 10_ms);

            const Eigen::MatrixXf X = dareSolver(discAB.first, discAB.second, Q, R);
            const Eigen::MatrixXf K = (R + discAB.second.transpose() * X * discAB.second).inverse() *
                                discAB.second.transpose() * X * discAB.first;

            const Eigen::Vector2f u = K * error;

            const DriveSpeeds speeds = {u.x(), u.y()};

            drivetrain->setDriveSpeeds(speeds, lastSpeeds);
        }
    }

    void end(bool interrupted) override { std::cout << "DONE" << std::endl; }

    bool isFinished() override { return motionProfile->getDuration() < pros::millis() * millisecond - startTime; }

    std::vector<Subsystem *> getRequirements() override { return {drivetrain}; }
};
