#pragma once

BEZIER_MIRRORED_MP_ASSET(n_1_6_1);
BEZIER_MIRRORED_MP_ASSET(n_1_6_2);

/**
 * Define elim autons
 */
class Negative {
private:
    static Command* n_6_ring_no_init(bool flip) {
        return new Sequence({
            new Ramsete(drivetrainSubsystem, flip ? &n_1_6_1_blue : &n_1_6_1_red),
            (new Rotate(drivetrainSubsystem, -50_deg, flip))->withTimeout(500_ms),
            new Ramsete(drivetrainSubsystem, flip ? &n_1_6_2_blue : &n_1_6_2_red),
            SharedCommands::descoreCorner(),
        });
    }
public:
    static Command* n_1_6() {
        Eigen::Vector3f startPose{(10.0_in).getValue(), (55.0_in).getValue(), (127_deg).getValue()};

        drivetrainSubsystem->updateAllianceColor(startPose);
        const bool flip = ALLIANCE != RED;

        return new Sequence({
            drivetrainSubsystem->setNorm(startPose.head<2>(), Eigen::Matrix2f::Identity() * 0.05, startPose.z(), flip),
            new ScheduleCommand(liftSubsystem->positionCommand(180_deg, 0.0)),
            SharedCommands::driveToAlliance(),
            n_6_ring_no_init(flip),
        });
    }

    static Command* n_6() {
        Eigen::Vector3f startPose{(10.0_in).getValue(), (55.0_in).getValue(), (127_deg).getValue()};

        drivetrainSubsystem->updateAllianceColor(startPose);
        const bool flip = ALLIANCE != RED;

        return new Sequence({
            drivetrainSubsystem->setNorm(startPose.head<2>(), Eigen::Matrix2f::Identity() * 0.05, startPose.z(), flip),
            n_6_ring_no_init(flip),
        });
    }
};
