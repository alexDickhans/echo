#pragma once

BEZIER_MIRRORED_MP_ASSET(safe_awp);
BEZIER_MIRRORED_MP_ASSET(sawp_1);

/**
 * Defines AWP autons
 */
class AWP {
public:
    /**
     * Adds a push to the same AWP
     * @return Command for auton
     */
    static Command *push_awp() {
        Eigen::Vector3f startPose{(12_in).getValue(), (60_in).getValue(), (-180_deg).getValue()};

        drivetrain->updateAllianceColor(startPose);
        const bool flip = ALLIANCE != RED;

        return new Sequence({
                drivetrain->setNorm(startPose.head<2>(), Eigen::Matrix2f::Identity() * 0.02, startPose.z(), flip),
                new ScheduleCommand(bottomIntake->movePct(0.0)->with(topIntake->pctCommand(0.0))),
                new TankMotionProfiling(drivetrain, {60_in / second, 100_in / second / second}, 12_in, flip, -180_deg,
                                        0.0),
                (new Rotate(drivetrain, -90_deg, flip, 0.0))->withTimeout(1.2_s),
                new ScheduleCommand(bottomIntake->movePct(-1.0)),
                new TankMotionProfiling(drivetrain, {60_in / second, 100_in / second / second}, -3_in, flip, -90_deg,
                                        0.0),
                new ScheduleCommand(SharedCommands::scoreAlliance()),
                (new Rotate(drivetrain, -90_deg, flip, -0.2, false))->withTimeout(0.3_s),
                new TankMotionProfiling(drivetrain, {60_in / second, 100_in / second / second}, 5_in, flip, -90_deg,
                                        0.0),
                (new Rotate(drivetrain, 0_deg, flip, 0.0))->withTimeout(0.8_s),
                new Ramsete(drivetrain, flip ? &safe_awp_blue : &safe_awp_red),
                drivetrain->pct(0.15, 0.15)->withTimeout(4.0_s),
        });
    }

    /**
     * Adds a push to the same AWP
     * @return Command for auton
     */
    static Command *sawp() {
        Eigen::Vector3f startPose{(-10_in).getValue(), (58_in).getValue(), (-13_deg).getValue()};

        drivetrain->updateAllianceColor(startPose);
        const bool flip = ALLIANCE != RED;

        return new Sequence({
                drivetrain->setNorm(startPose.head<2>(), Eigen::Matrix2f::Identity() * 0.02, startPose.z(), flip),
                new ScheduleCommand(
                        loadOneRingHigh->andThen(lift->moveToPosition(25_deg)->with(bottomIntake->movePct(0.0)->with(
                                TopIntakePositionCommand::fromReversePositionCommand(topIntake, -1.4, 0.0))))),
                new TankMotionProfiling(drivetrain, {60_in / second, 250_in / second / second}, 9.5_in, flip, -13_deg,
                                        0.0),
                (new Rotate(drivetrain, 90_deg, flip, 0.0))->withTimeout(2.0_s),
                new TankMotionProfiling(drivetrain, {60_in / second, 100_in / second / second}, 5_in, flip, 90_deg,
                                        0.0),
                (new ParallelCommandGroup({
                         bottomIntake->movePct(0.0),
                         lift->positionCommand(CONFIG::ALLIANCE_STAKE_SCORE_HEIGHT),
                         topIntake->pctCommand(-0.47),
                 }))
                        ->withTimeout(0.28_s)
                        ->andThen((new ParallelCommandGroup({
                                           bottomIntake->movePct(0.0),
                                           lift->positionCommand(0),
                                           topIntake->pctCommand(-1.0),
                                   }))
                                          ->withTimeout(0.1_s))
                        ->andThen((new ParallelCommandGroup({
                                           bottomIntake->movePct(0.0),
                                           lift->positionCommand(0),
                                           topIntake->pctCommand(1.0),
                                   }))
                                          ->withTimeout(0.6_s))
                        ->asProxy(),
                new Ramsete(drivetrain, flip ? &sawp_1_blue : &sawp_1_red),
                drivetrain->pct(0.15, 0.15)->withTimeout(4.0_s),
        });
    }
};
