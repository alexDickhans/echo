#pragma once

/**
 * Define elim autons
 */
class Negative {
public:
    static Command* n_1_6() {
        Eigen::Vector3f startPose{(10.0_in).getValue(), (55.6_in).getValue(), (90_deg).getValue()};

        drivetrainSubsystem->updateAllianceColor(startPose);
        const bool flip = ALLIANCE != RED;

        return new Sequence({
            drivetrainSubsystem->setNorm(startPose.head<2>(), Eigen::Matrix2f::Identity() * 0.05, startPose.z(), flip),
            new ScheduleCommand(liftSubsystem->positionCommand(180_deg, 0.0)),
            SharedCommands::arcOntoAlliance(flip, false),
        });
    }
};
