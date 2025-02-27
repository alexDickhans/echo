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
                drivetrainSubsystem->setNorm(Eigen::Vector2f(0.0, (60_in).getValue()), Eigen::Matrix2f::Identity() * 0.04,
                                    -90_deg, false),
                new ScheduleCommand(intakeOntoGoal->withTimeout(300_ms)),
                new WaitCommand(200_ms),
                new Ramsete(drivetrainSubsystem, &skills_1),
                drivetrainSubsystem->pct(0.1, 0.1)->withTimeout(0.2_s),
                drivetrainSubsystem->pct(0.1, 0.1)->race(
                        (new Command())),
                new Ramsete(drivetrainSubsystem, &skills_2),
                drivetrainSubsystem->pct(0.25, 0.25)->withTimeout(0.1_s),
                (new Command())
                        ->asProxy(),
                new Ramsete(drivetrainSubsystem, &skills_3),
                drivetrainSubsystem->pct(0.2, 0.2)->withTimeout(0.1_s),
                drivetrainSubsystem->pct(0.2, 0.2)->race(
                        (new Sequence({new Command()}))
                                ->asProxy()),
                new Ramsete(drivetrainSubsystem, &skills_4),
                new ScheduleCommand(hang),
        });
    }
};
