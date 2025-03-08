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
        return liftSubsystem->positionCommand(110_deg, 20_deg)->andThen((new TrapLiftPosition(liftSubsystem, 2.0_deg,
                                  TrapProfile({350.0, 1500.0}),
                                  TrapProfile::State(180.0, 0.0)))->withTimeout(400_ms));
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
