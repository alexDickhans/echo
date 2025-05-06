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
            drivetrainSubsystem->setNorm(Eigen::Vector2f((8_in).getValue(), (55.5_in).getValue()),
                                         Eigen::Matrix2f::Identity() * 0.04,
                                         128_deg, false),
            SharedCommands::scoreAlliance()->asProxy(),
            new ScheduleCommand(liftSubsystem->positionCommand(180_deg, 0.0)),
            new Ramsete(drivetrainSubsystem, &skills_1),
            drivetrainSubsystem->pct(0.25, 0.25)->race(
                SharedCommands::scoreWallStakes()->asProxy()),
            drivetrainSubsystem->pct(0.0, 0.0)->race(basicLoadLB->withTimeout(1500_ms)->asProxy()),
            drivetrainSubsystem->pct(0.4, 0.4)->race(
                SharedCommands::scoreWallStakes()->asProxy()),
            drivetrainSubsystem->pct(0.0, 0.0)->race(
                liftSubsystem->positionCommand(0.0)->withTimeout(150_ms)->asProxy()),
            new ScheduleCommand(loadLB),
            new Ramsete(drivetrainSubsystem, &skills_2),
            drivetrainSubsystem->pct(0.3, 0.3)->withTimeout(500_ms),
            new ScheduleCommand(liftSubsystem->positionCommand(CONFIG::WALL_STAKE_PRIME_HEIGHT, 0.0)->withTimeout(200_ms)->andThen(liftSubsystem->positionCommand(205_deg, 0.0))),
            new TankMotionProfiling(drivetrainSubsystem, {50_in/second, 140_in/second/second}, -4.9_in, false, 0.0, 0.0, false),
            new Ramsete(drivetrainSubsystem, &skills_3),
            drivetrainSubsystem->pct(0.25, 0.2)->withTimeout(0.1_s),
            drivetrainSubsystem->pct(0.2, 0.2)->race(
                SharedCommands::scoreWallStakesWithoutTopIntake()->asProxy())->andThen(
                new ScheduleCommand(liftSubsystem->positionCommand(155_deg, 0.0))),
            drivetrainSubsystem->pct(-0.5, -0.5)->withTimeout(0.27_s),
            new ScheduleCommand(intakeNoEject),
            new Ramsete(drivetrainSubsystem, &skills_4),
            drivetrainSubsystem->pct(0.25, 0.5)->withTimeout(0.3_s),
            new ScheduleCommand(hang),
        });
    }
};
