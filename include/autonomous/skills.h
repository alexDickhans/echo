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
                                         129_deg, false),
            SharedCommands::scoreAlliance()->asProxy(),
            new ScheduleCommand(liftSubsystem->positionCommand(180_deg, 0.0)),
            new Ramsete(drivetrainSubsystem, &skills_1),
            drivetrainSubsystem->pct(0.0, 0.0)->race(
                liftSubsystem->pctCommand(1.0)->withTimeout(550_ms)->asProxy()),
            drivetrainSubsystem->pct(0.1, 0.1)->race(loadLB->withTimeout(1200_ms)->asProxy()),
            drivetrainSubsystem->pct(0.0, 0.0)->race(
                liftSubsystem->pctCommand(1.0)->withTimeout(500_ms)->asProxy()),
            drivetrainSubsystem->pct(0.0, 0.0)->race(
                liftSubsystem->positionCommand(0.0)->withTimeout(100_ms)->asProxy()),
            new Ramsete(drivetrainSubsystem, &skills_2),
            drivetrainSubsystem->pct(0.0, 0.0)->withTimeout(300_ms),
            new Ramsete(drivetrainSubsystem, &skills_3),
            drivetrainSubsystem->pct(0.0, 0.0)->withTimeout(0.2_s),
            drivetrainSubsystem->pct(0.0, 0.0)->race(
                liftSubsystem->pctCommand(1.0)->withTimeout(500_ms)->asProxy())->andThen(
                new ScheduleCommand(liftSubsystem->positionCommand(90_deg, 0.0))),
            drivetrainSubsystem->pct(0.0, 0.0)->withTimeout(100_ms),
            new Ramsete(drivetrainSubsystem, &skills_4),
            // new ScheduleCommand(hang),
        });
    }
};
