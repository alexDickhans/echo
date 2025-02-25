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
                        (new Sequence({new ParallelRaceGroup({
                                               bottomIntakeSubsystem->movePctCommand(0.0),
                                               liftSubsystem->positionCommand(CONFIG::WALL_STAKE_SCORE_HEIGHT),
                                               topIntakeSubsystem->pctCommand(0.0),
                                       }),
                                       new ParallelRaceGroup({bottomIntakeSubsystem->movePctCommand(0.0),
                                                              liftSubsystem->positionCommand(CONFIG::WALL_STAKE_SCORE_HEIGHT),
                                                              topIntakeSubsystem->pctCommand(0.0), new WaitCommand(100_ms)}),
                                       (new ParallelCommandGroup({
                                                bottomIntakeSubsystem->movePctCommand(0.0),
                                                liftSubsystem->positionCommand(CONFIG::WALL_STAKE_SCORE_HEIGHT),
                                                topIntakeSubsystem->pctCommand(-1.0),
                                        }))
                                               ->withTimeout(0.6_s)}))
                                ->asProxy()),
                new Ramsete(drivetrainSubsystem, &skills_2),
                drivetrainSubsystem->pct(0.25, 0.25)->withTimeout(0.1_s),
                (new ParallelCommandGroup({
                         bottomIntakeSubsystem->movePctCommand(0.0),
                         liftSubsystem->positionCommand(CONFIG::ALLIANCE_STAKE_SCORE_HEIGHT),
                         topIntakeSubsystem->pctCommand(-0.47),
                 }))
                        ->withTimeout(0.28_s)
                        ->andThen((new ParallelCommandGroup({
                                           bottomIntakeSubsystem->movePctCommand(0.0),
                                           liftSubsystem->positionCommand(0),
                                           topIntakeSubsystem->pctCommand(-1.0),
                                   }))
                                          ->withTimeout(0.1_s))
                        ->andThen((new ParallelCommandGroup({
                                           bottomIntakeSubsystem->movePctCommand(0.0),
                                           liftSubsystem->positionCommand(0),
                                           topIntakeSubsystem->pctCommand(1.0),
                                   }))
                                          ->withTimeout(0.3_s))
                        ->asProxy(),
                new Ramsete(drivetrainSubsystem, &skills_3),
                drivetrainSubsystem->pct(0.2, 0.2)->withTimeout(0.1_s),
                drivetrainSubsystem->pct(0.2, 0.2)->race(
                        (new Sequence({new ParallelRaceGroup({
                                               bottomIntakeSubsystem->movePctCommand(0.0),
                                               liftSubsystem->positionCommand(CONFIG::WALL_STAKE_SCORE_HEIGHT),
                                               topIntakeSubsystem->pctCommand(0.0),
                                       }),
                                       new ParallelRaceGroup({bottomIntakeSubsystem->movePctCommand(0.0),
                                                              liftSubsystem->positionCommand(CONFIG::WALL_STAKE_SCORE_HEIGHT),
                                                              topIntakeSubsystem->pctCommand(0.0), new WaitCommand(100_ms)}),
                                       (new ParallelCommandGroup({
                                                bottomIntakeSubsystem->movePctCommand(0.0),
                                                liftSubsystem->positionCommand(CONFIG::WALL_STAKE_SCORE_HEIGHT),
                                                topIntakeSubsystem->pctCommand(-1.0),
                                        }))
                                               ->withTimeout(0.6_s)}))
                                ->asProxy()),
                new Ramsete(drivetrainSubsystem, &skills_4),
                new ScheduleCommand(hangSubsystem),
        });
    }
};
