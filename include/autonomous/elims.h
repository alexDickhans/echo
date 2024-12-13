#pragma once


BEZIER_MIRRORED_MP_ASSET(positive_1);
BEZIER_MIRRORED_MP_ASSET(positive_2);
BEZIER_MIRRORED_MP_ASSET(negative_1);
BEZIER_MIRRORED_MP_ASSET(negative_2);
BEZIER_MIRRORED_MP_ASSET(negative_3);
BEZIER_MIRRORED_MP_ASSET(negative_2_no_alliance);

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
                new ScheduleCommand(bottomIntake->movePct(-1.0)->with(topIntake->pctCommand(0.0))),
                new TankMotionProfiling(drivetrain, {62_in / second, 100_in / second / second}, 12_in, flip, 0_deg,
                                        0.0),
                (new Rotate(drivetrain, -90_deg, flip, 0.0))->withTimeout(1.2_s),
                new ScheduleCommand(bottomIntake->movePct(-1.0)),
                new TankMotionProfiling(drivetrain, {65_in / second, 100_in / second / second}, -4_in, flip, -90_deg,
                                        0.0),
                new ScheduleCommand(SharedCommands::scoreAlliance()),
                (new Rotate(drivetrain, -90_deg, flip, -1000, false))->withTimeout(1.0_s),
                new TankMotionProfiling(drivetrain, {62_in / second, 100_in / second / second}, 5_in, flip, -90_deg,
                                        0.0),
                (new Rotate(drivetrain, 180_deg, flip, 0.0))->withTimeout(0.8_s),
                new Ramsete(drivetrain, flip ? &positive_1_blue : &positive_1_red),
                SharedCommands::descoreCorner(),
                new Ramsete(drivetrain, flip ? &positive_2_blue : &positive_2_red),
                drivetrain->pct(0.15, 0.15)->withTimeout(2.0_s),
        });
    }

    /**
     * Negative corner eliminations auton
     *
     * @return Command for auton
     */
    static Command *neg_elim() {
        Eigen::Vector3f startPose{(36_in).getValue(), (40_in).getValue(), (-90_deg).getValue()};

        drivetrain->updateAllianceColor(startPose);
        const bool flip = ALLIANCE != RED;

        return new Sequence({
                drivetrain->setNorm(startPose.head<2>(), Eigen::Matrix2f::Identity() * 0.05, startPose.z(), flip),
                new ScheduleCommand(loadOneRingLow->andThen(loadOneRingLow)
                                            ->andThen(bottomIntake->movePct(0.0)->with(topIntake->pctCommand(0.0)))),
                new Ramsete(drivetrain, flip ? &negative_1_blue : &negative_1_red),
                SharedCommands::descoreCorner(),
                new Ramsete(drivetrain, flip ? &negative_2_blue : &negative_2_red),
                (new ParallelCommandGroup({
                         bottomIntake->movePct(0.0),
                         lift->positionCommand(CONFIG::WALL_STAKE_SCORE_HEIGHT),
                         topIntake->pctCommand(-1.0),
                 }))
                        ->asProxy()
                        ->with(drivetrain->pct(0.3, 0.3))
                        ->withTimeout(0.7_s),
                new Ramsete(drivetrain, flip ? &negative_3_blue : &negative_3_red),
        });
    }

    static Command *neg_elim_no_alliance() {
        Eigen::Vector3f startPose{(36_in).getValue(), (40_in).getValue(), (-90_deg).getValue()};

        drivetrain->updateAllianceColor(startPose);
        const bool flip = ALLIANCE != RED;

        return new Sequence({
                drivetrain->setNorm(startPose.head<2>(), Eigen::Matrix2f::Identity() * 0.05, startPose.z(), flip),
                new ScheduleCommand(loadOneRingLow->andThen(loadOneRingLow)
                                            ->andThen(bottomIntake->movePct(0.0)->with(topIntake->pctCommand(0.0)))),
                new Ramsete(drivetrain, flip ? &negative_1_blue : &negative_1_red),
                SharedCommands::descoreCorner(),
                new Ramsete(drivetrain, flip ? &negative_2_no_alliance_blue : &negative_2_no_alliance_red),
                drivetrain->pct(0.15, 0.15)->withTimeout(4.0_s),
        });
    }
};
