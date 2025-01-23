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
                drivetrain->setNorm(Eigen::Vector2f(0.0, (60_in).getValue()), Eigen::Matrix2f::Identity() * 0.04,
                                    -90_deg, false),
                new ScheduleCommand(intakeOntoGoal->withTimeout(300_ms)),
                new WaitCommand(200_ms),
                new Ramsete(drivetrain, &skills_1),
                drivetrain->pct(0.1, 0.1)->withTimeout(0.2_s),
                drivetrain->pct(0.1, 0.1)->race(
                        (new Sequence({new ParallelRaceGroup({
                                               bottomIntake->movePct(0.0),
                                               lift->moveToPosition(CONFIG::WALL_STAKE_SCORE_HEIGHT),
                                               topIntake->pctCommand(0.0),
                                       }),
                                       new ParallelRaceGroup({
                                               bottomIntake->movePct(0.0),
                                               lift->moveToPosition(CONFIG::WALL_STAKE_SCORE_HEIGHT),
                                               topIntake->pctCommand(0.0),
                                       }),
                                       (new ParallelCommandGroup({
                                                bottomIntake->movePct(0.0),
                                                lift->positionCommand(CONFIG::WALL_STAKE_SCORE_HEIGHT),
                                                topIntake->pctCommand(-1.0),
                                        }))
                                               ->withTimeout(0.8_s)}))
                                ->asProxy()),
                new Ramsete(drivetrain, &skills_2),
                drivetrain->pct(0.25, 0.25)->withTimeout(0.1_s),
                (new ParallelCommandGroup({
                         bottomIntake->movePct(0.0),
                         lift->positionCommand(CONFIG::ALLIANCE_STAKE_SCORE_HEIGHT),
                         topIntake->pctCommand(-0.47),
                 }))
                        ->withTimeout(0.28_s)
                        ->andThen((new ParallelCommandGroup({
                                           bottomIntake->movePct(0.0),
                                           lift->positionCommand(0),
                                           topIntake->pctCommand(-1.0),
                                   }))
                                          ->withTimeout(0.1_s))
                        ->andThen((new ParallelCommandGroup({
                                           bottomIntake->movePct(0.0),
                                           lift->positionCommand(0),
                                           topIntake->pctCommand(1.0),
                                   }))
                                          ->withTimeout(0.3_s))
                        ->asProxy(),
                new Ramsete(drivetrain, &skills_3),
                drivetrain->pct(0.1, 0.1)->withTimeout(0.2_s),
                drivetrain->pct(0.1, 0.1)->race(
                        (new Sequence({new ParallelRaceGroup({
                                               bottomIntake->movePct(0.0),
                                               lift->moveToPosition(CONFIG::WALL_STAKE_SCORE_HEIGHT),
                                               topIntake->pctCommand(0.0),
                                       }),
                                       (new ParallelCommandGroup({
                                                bottomIntake->movePct(0.0),
                                                lift->positionCommand(CONFIG::WALL_STAKE_SCORE_HEIGHT),
                                                topIntake->pctCommand(-1.0),
                                        }))
                                               ->withTimeout(1.0_s)}))
                                ->asProxy()),
                new Ramsete(drivetrain, &skills_4),
                new ScheduleCommand(hang),
        });
    }
};
