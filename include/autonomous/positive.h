#pragma once

class Positive {
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
            SharedCommands::driveToAlliance(),
        });
    }

    static Command *pos_elim_no_alliance() {
        Eigen::Vector3f startPose{(-36.0_in).getValue(), (56_in).getValue(), (90_deg).getValue()};

        drivetrainSubsystem->updateAllianceColor(startPose);
        const bool flip = ALLIANCE != RED;

        return new Sequence({
            drivetrainSubsystem->setNorm(startPose.head<2>(), Eigen::Matrix2f::Identity() * 0.05, startPose.z(), flip),
            liftSubsystem->positionCommand(50_deg)->withTimeout(50_ms)->asProxy(),
        });
    }
};
