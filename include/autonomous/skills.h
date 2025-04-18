#pragma once

#include "sharedCommands.h"
#include "subsystems/subsystems.h"

/**
 * Add motion profiled path assets
 */
BEZIER_MP_ASSET(skills_1);
BEZIER_MP_ASSET(skills_2);
BEZIER_MP_ASSET(skills_3);
BEZIER_MP_ASSET(skills_4);

/**
 * Skills autonomous
 */
class Skills {
public:
    /**
     * Build skills command
     *
     * @return Command that runs skills
     */
    static Command *skills() {
        return new Sequence({
            drivetrainSubsystem->setNorm(Eigen::Vector2f((6_in).getValue(), (55.5_in).getValue()),
                                         Eigen::Matrix2f::Identity() * 0.04,
                                         128_deg, false),
            SharedCommands::scoreAlliance()->asProxy(),
            new ScheduleCommand(liftSubsystem->positionCommand(180_deg, 0.0)),
            new Ramsete(drivetrainSubsystem, &skills_1),
            drivetrainSubsystem->pct(0.4, 0.4)->race(
                SharedCommands::scoreWallStakes()->asProxy()),
            drivetrainSubsystem->pct(0.0, 0.0)->race(basicLoadLB->withTimeout(1500_ms)->asProxy()),
            drivetrainSubsystem->pct(0.4, 0.4)->race(
                SharedCommands::scoreWallStakes()->asProxy()),
            drivetrainSubsystem->pct(0.0, 0.0)->race(
                liftSubsystem->positionCommand(0.0)->withTimeout(150_ms)->asProxy()),
            new ScheduleCommand(loadLB),
            new Ramsete(drivetrainSubsystem, &skills_2),
            (new Rotate(drivetrainSubsystem, -90_deg, false))->withTimeout(150_ms),
            new Ramsete(drivetrainSubsystem, &skills_3),
            drivetrainSubsystem->pct(0.4, 0.4)->withTimeout(0.2_s),
            drivetrainSubsystem->pct(0.4, 0.4)->race(
                SharedCommands::scoreWallStakes()->asProxy())->andThen(
                new ScheduleCommand(liftSubsystem->positionCommand(90_deg, 0.0))),
            drivetrainSubsystem->pct(0.0, 0.0)->withTimeout(140_ms),
            new Ramsete(drivetrainSubsystem, &skills_4),
            // new ScheduleCommand(hang),
        });
    }
};
