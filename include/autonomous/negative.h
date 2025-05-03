#pragma once

BEZIER_MP_ASSET(n_1_6_1_red);
BEZIER_MP_ASSET(n_1_6_1_blue);
BEZIER_MP_ASSET(n_1_6_2_red);
BEZIER_MP_ASSET(n_1_6_2_blue);
BEZIER_MP_ASSET(n_1_6_3_red);
BEZIER_MP_ASSET(n_1_6_3_blue);
BEZIER_MP_ASSET(n_1_6_4_red);
BEZIER_MP_ASSET(n_1_6_4_blue);

/**
 * Define elim autons
 */
class Negative {
private:
    static Command *n_6_ring_no_init(bool flip) {
        return new Sequence({
            new Ramsete(drivetrainSubsystem, flip ? &n_1_6_1_blue : &n_1_6_1_red),
            (new Rotate(drivetrainSubsystem, -30_deg, flip))->withTimeout(500_ms),
            new Ramsete(drivetrainSubsystem, flip ? &n_1_6_2_blue : &n_1_6_2_red),
            SharedCommands::descoreCorner(),
            new ScheduleCommand(intakeWithEject),
            new Ramsete(drivetrainSubsystem, flip ? &n_1_6_3_blue : &n_1_6_3_red),
            drivetrainSubsystem->pct(0.07, 0.07),
        });
    }

public:
    static Command *n_1_6() {
        Eigen::Vector3f startPose{(10.3_in).getValue(), (54.7_in).getValue(), (127_deg).getValue()};

        drivetrainSubsystem->updateAllianceColor(startPose);
        const bool flip = ALLIANCE != RED;

        return new Sequence({
            drivetrainSubsystem->setNorm(startPose.head<2>(), Eigen::Matrix2f::Identity() * 0.05, startPose.z(), flip),
            new ScheduleCommand(liftSubsystem->positionCommand(190_deg, 0.0)),
            SharedCommands::driveToAlliance(),
            n_6_ring_no_init(flip),
        });
    }

    static Command *n_1_6p() {
        Eigen::Vector3f startPose{(10.3_in).getValue(), (54.7_in).getValue(), (127_deg).getValue()};

        drivetrainSubsystem->updateAllianceColor(startPose);
        const bool flip = ALLIANCE != RED;

        return new Sequence({
            drivetrainSubsystem->setNorm(startPose.head<2>(), Eigen::Matrix2f::Identity() * 0.05, startPose.z(), flip),
            new ScheduleCommand(liftSubsystem->positionCommand(190_deg, 0.0)),
            SharedCommands::driveToAlliance(),
            new Ramsete(drivetrainSubsystem, flip ? &n_1_6_1_blue : &n_1_6_1_red),
            (new Rotate(drivetrainSubsystem, -30_deg, flip))->withTimeout(500_ms),
            new Ramsete(drivetrainSubsystem, flip ? &n_1_6_4_blue : &n_1_6_4_red),
            SharedCommands::descoreCornerFull(),
            new ScheduleCommand(intakeWithEject),
            drivetrainSubsystem->pct(-1.0, -1.0)->withTimeout(80_ms),
            new Rotate(drivetrainSubsystem, -135_deg, true, 0),
        });
    }

    static Command *n_6() {
        Eigen::Vector3f startPose{(10.3_in).getValue(), (54.7_in).getValue(), (127_deg).getValue()};

        drivetrainSubsystem->updateAllianceColor(startPose);
        const bool flip = ALLIANCE != RED;

        return new Sequence({
            drivetrainSubsystem->setNorm(startPose.head<2>(), Eigen::Matrix2f::Identity() * 0.05, startPose.z(), flip),
            n_6_ring_no_init(flip),
        });
    }
};
