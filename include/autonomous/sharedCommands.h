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
        return new Sequence({
            new ParallelRaceGroup({
                bottomIntakeSubsystem->movePctCommand(0.0),
                liftSubsystem->positionCommand(7_deg),
                topIntakeSubsystem->pctCommand(0.0),
            }),
            new ParallelRaceGroup({
                bottomIntakeSubsystem->movePctCommand(0.0),
                liftSubsystem->positionCommand(7_deg),
                topIntakeSubsystem->pctCommand(1.0),
                new WaitCommand(10_ms),
            }),
            (new ParallelCommandGroup({
                bottomIntakeSubsystem->movePctCommand(0.0),
                liftSubsystem->positionCommand(0_deg),
                topIntakeSubsystem->pctCommand(1.0),
            }))
            ->withTimeout(500_ms)
        });
    }

    /**
     * Command to get rings out of the corner
     *
     * @return Command that removes rings from the corner
     */
    static Command *descoreCorner() {
        return new Sequence({
            drivetrainSubsystem->pct(0.15, 0.15)->withTimeout(150_ms),
            new TankMotionProfiling(drivetrainSubsystem, {15_in / second, 70_in / second / second}, -7_in,
                                    false, -90_deg, 0.0, false),
            new TankMotionProfiling(drivetrainSubsystem, {40_in / second, 140_in / second / second}, 7.0_in,
                                    false, -90_deg, 0.0, false),
            drivetrainSubsystem->pct(0.15, 0.15)->withTimeout(150_ms),
            new TankMotionProfiling(drivetrainSubsystem, {15_in / second, 80_in / second / second}, -7_in,
                                    false, -90_deg, 0.0, false),
            new TankMotionProfiling(drivetrainSubsystem, {40_in / second, 140_in / second / second}, 7.0_in, false,
                                    -90_deg, 0.0, false),
            drivetrainSubsystem->pct(0.15, 0.15)->withTimeout(150_ms),
        });
    }
};
