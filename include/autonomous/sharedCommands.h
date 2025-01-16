#pragma once

#include "subsystems/subsystems.h"

/**
 * Commands that are shared between multiple skills or auton routines
 */
class SharedCommands {
public:
    /**
     * Build command to score on alliance stakes
     *
     * @return Command that scores on alliance
     */
    static Command *scoreAlliance() {
        return (new ParallelCommandGroup({
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
                                  ->withTimeout(0.6_s))
                ->asProxy();
    }


    /**
     * Build command to score on alliance stakes, but throw the ring
     * slightly less
     *
     * @return Command that scores on alliance
     */
    static Command *scoreAlliance2() {
        return new Sequence({new ParallelRaceGroup({
                                     bottomIntake->movePct(0.0),
                                     lift->moveToPosition(7_deg),
                                     topIntake->pctCommand(0.0),
                             }),
                             new ParallelRaceGroup({
                                     bottomIntake->movePct(0.0),
                                     lift->positionCommand(7_deg),
                                     topIntake->pctCommand(1.0),
                                     new WaitCommand(10_ms),
                             }),
                             (new ParallelCommandGroup({
                                      bottomIntake->movePct(0.0),
                                      lift->positionCommand(0_deg),
                                      topIntake->pctCommand(1.0),
                              }))
                                     ->withTimeout(500_ms)});
    }

    /**
     * Command to get rings out of the corner
     *
     * @return Command that removes rings from the corner
     */
    static Command *descoreCorner() {
        return new Sequence({new TankMotionProfiling(drivetrain, {10_in / second, 70_in / second / second}, -7_in,
                                                     false, -90_deg, 0.0, false),
                             new TankMotionProfiling(drivetrain, {35_in / second, 80_in / second / second}, 7.3_in,
                                                     false, -90_deg, 0.0, false),
                             new TankMotionProfiling(drivetrain, {35_in / second, 80_in / second / second}, -7_in,
                                                     false, -90_deg, 0.0, false),
                             new TankMotionProfiling(drivetrain, {35_in / second, 70_in / second / second}, 7.3_in, false,
                                                     -90_deg, 0.0, false)});
    }
};
