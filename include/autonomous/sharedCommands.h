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
        return new Sequence({new ParallelRaceGroup({
                                     bottomIntake->movePct(0.0),
                                     lift->moveToPosition(8_deg),
                                     topIntake->pctCommand(0.0),
                             }),
                             new ParallelRaceGroup({
                                     bottomIntake->movePct(0.0),
                                     lift->positionCommand(8_deg),
                                     topIntake->pctCommand(1.0),
                                     new WaitCommand(30_ms),
                             }),
                             (new ParallelCommandGroup({
                                      bottomIntake->movePct(0.0),
                                      lift->positionCommand(0_deg),
                                      topIntake->pctCommand(1.0),
                              }))
                                     ->withTimeout(500_ms)});
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
        // TODO
    }
};
