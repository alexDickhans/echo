#pragma once


BEZIER_MIRRORED_MP_ASSET(positive_1);
BEZIER_MIRRORED_MP_ASSET(positive_1_no_alliance);
BEZIER_MIRRORED_MP_ASSET(positive_2);
BEZIER_MP_ASSET(negative_1_red);
BEZIER_MP_ASSET(negative_1_blue);
BEZIER_MIRRORED_MP_ASSET(negative_2);
BEZIER_MIRRORED_MP_ASSET(negative_2_pole_touch);

/**
 * Define elim autons
 */
class Elims {
public:
    /**
     * Positive corner eliminations auton
     *
     * @return Command for auton
     */
    static Command *pos_elim() {
        Eigen::Vector3f startPose{(-10.0_in).getValue(), (55.6_in).getValue(), (90_deg).getValue()};

        drivetrainSubsystem->updateAllianceColor(startPose);
        const bool flip = ALLIANCE != RED;

        return new Sequence({
            drivetrainSubsystem->setNorm(startPose.head<2>(), Eigen::Matrix2f::Identity() * 0.05, startPose.z(), flip),
            new ScheduleCommand(liftSubsystem->positionCommand(180_deg, 0.0)),
            SharedCommands::arcOntoAlliance(flip, true),
            new Ramsete(drivetrainSubsystem, flip ? &positive_1_blue : &positive_1_red),
            SharedCommands::descoreCorner(),
            new Ramsete(drivetrainSubsystem, flip ? &positive_2_blue : &positive_2_red),
            drivetrainSubsystem->pct(0.30, 0.30)->withTimeout(10.0_s),
        });
    }

    static Command *pos_elim_no_alliance() {
        Eigen::Vector3f startPose{(-10.0_in).getValue(), (55.6_in).getValue(), (90_deg).getValue()};

        drivetrainSubsystem->updateAllianceColor(startPose);
        const bool flip = ALLIANCE != RED;

        return new Sequence({
            drivetrainSubsystem->setNorm(startPose.head<2>(), Eigen::Matrix2f::Identity() * 0.05, startPose.z(), flip),
            liftSubsystem->positionCommand(50_deg)->withTimeout(50_ms)->asProxy(),
            new Ramsete(drivetrainSubsystem, flip ? &positive_1_no_alliance_blue : &positive_1_no_alliance_red),
            SharedCommands::descoreCorner(),
            new Ramsete(drivetrainSubsystem, flip ? &positive_2_blue : &positive_2_red),
            drivetrainSubsystem->pct(0.30, 0.30)->withTimeout(10.0_s),
        });
    }

    /**
     * Negative corner eliminations auton
     *
     * @return Command for auton
     */
    static Command *neg_elim() {
        Eigen::Vector3f startPose{(10.0_in).getValue(), (55.6_in).getValue(), (90_deg).getValue()};

        drivetrainSubsystem->updateAllianceColor(startPose);
        const bool flip = ALLIANCE != RED;

        return new Sequence({
            drivetrainSubsystem->setNorm(startPose.head<2>(), Eigen::Matrix2f::Identity() * 0.05, startPose.z(), flip),
            new ScheduleCommand(liftSubsystem->positionCommand(180_deg, 0.0)),
            SharedCommands::arcOntoAlliance(flip, false),
            new Ramsete(drivetrainSubsystem, flip ? &negative_1_blue : &negative_1_red),
            SharedCommands::descoreCorner(),
            new Ramsete(drivetrainSubsystem, flip ? &negative_2_blue : &negative_2_red),
        });
    }

    /**
     * Negative corner eliminations auton
     *
     * @return Command for auton
     */
    static Command *neg_elim_pole_touch() {
        Eigen::Vector3f startPose{(12_in).getValue(), (60_in).getValue(), (180_deg).getValue()};

        drivetrainSubsystem->updateAllianceColor(startPose);
        const bool flip = ALLIANCE != RED;

        return new Sequence({
            drivetrainSubsystem->setNorm(startPose.head<2>(), Eigen::Matrix2f::Identity() * 0.05, startPose.z(), flip),
            new ScheduleCommand(liftSubsystem->positionCommand(180_deg, 0.0)),
            SharedCommands::arcOntoAlliance(flip, false),
            new Ramsete(drivetrainSubsystem, flip ? &negative_1_blue : &negative_1_red),
            SharedCommands::descoreCorner(),
            new Ramsete(drivetrainSubsystem, flip ? &negative_2_pole_touch_blue : &negative_2_pole_touch_red),
            drivetrainSubsystem->pct(0.30, 0.30)->withTimeout(4.0_s),
        });
    }
};
