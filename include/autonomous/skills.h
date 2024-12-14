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
                SharedCommands::scoreAlliance()->asProxy(),
                new Ramsete(drivetrain, &skills_1),
                drivetrain->pct(0, 0)->withTimeout(0.3_s),
                drivetrain->pct(0.15, 0.15)
                        ->race((new Sequence({new ParallelRaceGroup({
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
                (new ParallelCommandGroup({
                         bottomIntake->movePct(0.0),
                         lift->positionCommand(CONFIG::WALL_STAKE_SCORE_HEIGHT),
                         topIntake->pctCommand(-1.0),
                 }))
                        ->asProxy()
                        ->with(drivetrain->pct(0.3, 0.3))
                        ->withTimeout(0.5_s),
                new Ramsete(drivetrain, &skills_3),
                drivetrain->pct(0.15, 0.15)
                        ->race((new Sequence({
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
                                                      ->withTimeout(1.0_s)}))
                                       ->asProxy()),
                new Ramsete(drivetrain, &skills_4),
                new ScheduleCommand(hang),
        });
    }
};
