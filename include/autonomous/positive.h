#pragma once

class Positive {
public:
    static Command *p_4() {
        Eigen::Vector3f startPose{(10.0_in).getValue(), (55.0_in).getValue(), (127_deg).getValue()};

        drivetrainSubsystem->updateAllianceColor(startPose);
        const bool flip = ALLIANCE != RED;

        return new Sequence({
            drivetrainSubsystem->setNorm(startPose.head<2>(), Eigen::Matrix2f::Identity() * 0.05, startPose.z(), flip),
        });
    }

    static Command *p_1_3goal() {
        Eigen::Vector3f startPose{(10.0_in).getValue(), (55.0_in).getValue(), (127_deg).getValue()};

        drivetrainSubsystem->updateAllianceColor(startPose);
        const bool flip = ALLIANCE != RED;

        return new Sequence({
            drivetrainSubsystem->setNorm(startPose.head<2>(), Eigen::Matrix2f::Identity() * 0.05, startPose.z(), flip),
        });
    }

    static Command *p_1_3alliance() {
        Eigen::Vector3f startPose{(10.0_in).getValue(), (55.0_in).getValue(), (127_deg).getValue()};

        drivetrainSubsystem->updateAllianceColor(startPose);
        const bool flip = ALLIANCE != RED;

        return new Sequence({
            drivetrainSubsystem->setNorm(startPose.head<2>(), Eigen::Matrix2f::Identity() * 0.05, startPose.z(), flip),
            new ScheduleCommand(liftSubsystem->positionCommand(180_deg, 0.0)),
            SharedCommands::driveToAlliance(),
        });
    }

    static Command *p_1_1_2() {
        Eigen::Vector3f startPose{(10.0_in).getValue(), (55.0_in).getValue(), (127_deg).getValue()};

        drivetrainSubsystem->updateAllianceColor(startPose);
        const bool flip = ALLIANCE != RED;

        return new Sequence({
            drivetrainSubsystem->setNorm(startPose.head<2>(), Eigen::Matrix2f::Identity() * 0.05, startPose.z(), flip),
            new ScheduleCommand(liftSubsystem->positionCommand(180_deg, 0.0)),
            SharedCommands::driveToAlliance(),
        });
    }
};
