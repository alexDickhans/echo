#pragma once

BEZIER_MIRRORED_MP_ASSET(p_4_1);
BEZIER_MIRRORED_MP_ASSET(p_4_2);
BEZIER_MIRRORED_MP_ASSET(p_4_3);
BEZIER_MIRRORED_MP_ASSET(p_1_3goal_1);
BEZIER_MIRRORED_MP_ASSET(p_1_1_2);

class Positive {
public:
    static Command *p_4() {
        Eigen::Vector3f startPose{(10.0_in).getValue(), (55.0_in).getValue(), (127_deg).getValue()};

        drivetrainSubsystem->updateAllianceColor(startPose);
        const bool flip = ALLIANCE != RED;

        return new Sequence({
            drivetrainSubsystem->setNorm(startPose.head<2>(), Eigen::Matrix2f::Identity() * 0.05, startPose.z(), flip),
            new Ramsete(drivetrainSubsystem, flip ? &p_4_1_blue : &p_4_1_red),
            (new Rotate(drivetrainSubsystem, 170_deg, flip))->withTimeout(500_ms),
            new Ramsete(drivetrainSubsystem, flip ? &p_4_2_blue : &p_4_2_red),
            SharedCommands::descoreCorner(),
            new Ramsete(drivetrainSubsystem, flip ? &p_4_3_blue : &p_4_3_red),
            new ScheduleCommand(liftSubsystem->pctCommand(-0.2)),
        });
    }

    static Command *p_1_3goal() {
        Eigen::Vector3f startPose{(10.0_in).getValue(), (55.0_in).getValue(), (127_deg).getValue()};

        drivetrainSubsystem->updateAllianceColor(startPose);
        const bool flip = ALLIANCE != RED;

        return new Sequence({
            drivetrainSubsystem->setNorm(startPose.head<2>(), Eigen::Matrix2f::Identity() * 0.05, startPose.z(), flip),
            new Ramsete(drivetrainSubsystem, flip ? &p_1_3goal_1_blue : &p_1_3goal_1_red),
            SharedCommands::descoreCorner(),
            new Ramsete(drivetrainSubsystem, flip ? &p_4_3_blue : &p_4_3_red),
            new ScheduleCommand(liftSubsystem->pctCommand(-0.2)),
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
            new Ramsete(drivetrainSubsystem, flip ? &p_4_1_blue : &p_4_1_red),
            (new Rotate(drivetrainSubsystem, 170_deg, flip))->withTimeout(500_ms),
            new Ramsete(drivetrainSubsystem, flip ? &p_4_2_blue : &p_4_2_red),
            SharedCommands::descoreCorner(),
            new Ramsete(drivetrainSubsystem, flip ? &p_4_3_blue : &p_4_3_red),
            new ScheduleCommand(liftSubsystem->pctCommand(-0.2)),
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
            new Ramsete(drivetrainSubsystem, flip ? &p_1_1_2_blue : &p_1_1_2_red),
            new Ramsete(drivetrainSubsystem, flip ? &p_1_3goal_1_blue : &p_1_3goal_1_red),
            SharedCommands::descoreCorner(),
            new Ramsete(drivetrainSubsystem, flip ? &p_4_3_blue : &p_4_3_red),
            new ScheduleCommand(liftSubsystem->pctCommand(-0.2)),
        });
    }
};
