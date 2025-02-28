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
        Eigen::Vector3f startPose{(12_in).getValue(), (60_in).getValue(), (180_deg).getValue()};

        drivetrainSubsystem->updateAllianceColor(startPose);
        const bool flip = ALLIANCE != RED;

        return new Sequence({
                drivetrainSubsystem->setNorm(startPose.head<2>(), Eigen::Matrix2f::Identity() * 0.05, startPose.z(), flip),
                new ScheduleCommand(liftSubsystem->positionCommand(50_deg)->withTimeout(500_ms)),
                new TankMotionProfiling(drivetrainSubsystem, {60_in / second, 100_in / second / second}, 12_in, flip, 180_deg),
                new Ramsete(drivetrainSubsystem, flip ? &safe_awp_blue : &safe_awp_red),
                drivetrainSubsystem->pct(0.15, 0.15)->withTimeout(4.0_s),
        });
    }

    /**
     * Adds a push to the same AWP
     * @return Command for auton
     */
    static Command *sawp() {
        Eigen::Vector3f startPose{(12_in).getValue(), (60_in).getValue(), (180_deg).getValue()};

        drivetrainSubsystem->updateAllianceColor(startPose);
        const bool flip = ALLIANCE != RED;

        return new Sequence({
                drivetrainSubsystem->setNorm(startPose.head<2>(), Eigen::Matrix2f::Identity() * 0.05, startPose.z(), flip),
                new ScheduleCommand(liftSubsystem->positionCommand(50_deg)->withTimeout(500_ms)),
                new TankMotionProfiling(drivetrainSubsystem, {60_in / second, 100_in / second / second}, 14_in, flip, 180_deg),
                new Ramsete(drivetrainSubsystem, flip ? &sawp_1_blue : &sawp_1_red),
                drivetrainSubsystem->pct(0.15, 0.15)->withTimeout(4.0_s),
        });
    }
};
