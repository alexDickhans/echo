#pragma once


BEZIER_MIRRORED_MP_ASSET(positive_1);
BEZIER_MIRRORED_MP_ASSET(positive_1_no_alliance);
BEZIER_MIRRORED_MP_ASSET(positive_2);
BEZIER_MIRRORED_MP_ASSET(negative_1);
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
        Eigen::Vector3f startPose{(-12_in).getValue(), (60_in).getValue(), (0_deg).getValue()};

        drivetrain->updateAllianceColor(startPose);
        const bool flip = ALLIANCE != RED;

        return new Sequence({
            drivetrain->setNorm(startPose.head<2>(), Eigen::Matrix2f::Identity() * 0.05, startPose.z(), flip),
            lift->positionCommand(50_deg)->withTimeout(50_ms)->asProxy(),
            new ScheduleCommand(lift->positionCommand(50_deg)->withTimeout(530_ms)),
            new TankMotionProfiling(drivetrain, {60_in / second, 100_in / second / second}, 14_in, flip, 0_deg),
            new Ramsete(drivetrain, flip ? &positive_1_blue : &positive_1_red),
            SharedCommands::descoreCorner(),
            new Ramsete(drivetrain, flip ? &positive_2_blue : &positive_2_red),
            drivetrain->pct(0.30, 0.30)->withTimeout(4.0_s),
        });
    }

    static Command *pos_elim_no_alliance() {
        Eigen::Vector3f startPose{(-12_in).getValue(), (60_in).getValue(), (0_deg).getValue()};

        drivetrain->updateAllianceColor(startPose);
        const bool flip = ALLIANCE != RED;

        return new Sequence({
            drivetrain->setNorm(startPose.head<2>(), Eigen::Matrix2f::Identity() * 0.05, startPose.z(), flip),
            lift->positionCommand(50_deg)->withTimeout(50_ms)->asProxy(),
            new Ramsete(drivetrain, flip ? &positive_1_no_alliance_blue : &positive_1_no_alliance_red),
            SharedCommands::descoreCorner(),
            new Ramsete(drivetrain, flip ? &positive_2_blue : &positive_2_red),
            drivetrain->pct(0.30, 0.30)->withTimeout(4.0_s),
        });
    }

    /**
     * Negative corner eliminations auton
     *
     * @return Command for auton
     */
    static Command *neg_elim() {
        Eigen::Vector3f startPose{(12_in).getValue(), (60_in).getValue(), (180_deg).getValue()};

        drivetrain->updateAllianceColor(startPose);
        const bool flip = ALLIANCE != RED;

        return new Sequence({
            drivetrain->setNorm(startPose.head<2>(), Eigen::Matrix2f::Identity() * 0.05, startPose.z(), flip),
            new ScheduleCommand(lift->positionCommand(50_deg)->withTimeout(500_ms)),
            new TankMotionProfiling(drivetrain, {60_in / second, 100_in / second / second}, 12_in, flip, 180_deg),
            new Ramsete(drivetrain, flip ? &negative_1_blue : &negative_1_red),
            SharedCommands::descoreCorner(),
            new Ramsete(drivetrain, flip ? &negative_2_blue : &negative_2_red),
        });
    }

    /**
     * Negative corner eliminations auton
     *
     * @return Command for auton
     */
    static Command *neg_elim_pole_touch() {
        Eigen::Vector3f startPose{(12_in).getValue(), (60_in).getValue(), (180_deg).getValue()};

        drivetrain->updateAllianceColor(startPose);
        const bool flip = ALLIANCE != RED;

        return new Sequence({
            drivetrain->setNorm(startPose.head<2>(), Eigen::Matrix2f::Identity() * 0.05, startPose.z(), flip),
            lift->positionCommand(50_deg)->withTimeout(50_ms)->asProxy(),
            new ScheduleCommand(lift->positionCommand(50_deg)->withTimeout(530_ms)),
            new TankMotionProfiling(drivetrain, {60_in / second, 100_in / second / second}, 14_in, flip, 180_deg),
            new Ramsete(drivetrain, flip ? &negative_1_blue : &negative_1_red),
            SharedCommands::descoreCorner(),
            new Ramsete(drivetrain, flip ? &negative_2_pole_touch_blue : &negative_2_pole_touch_red),
            drivetrain->pct(0.30, 0.30)->withTimeout(4.0_s),
        });
    }
};
