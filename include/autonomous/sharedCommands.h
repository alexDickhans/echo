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
    static Command *scoreAlliance() { return liftSubsystem->positionCommand(180_deg, 20_deg)->withTimeout(400_ms); }

    static Command *scoreWallStakes() {
        return liftSubsystem->pctCommand(1.0)
            ->until([]() { return liftSubsystem->getPosition() > 155_deg; })
            ->withTimeout(700_ms);
    }

    static Command *driveToAlliance() {
        return new TankMotionProfiling(drivetrainSubsystem, {20_in / second, 100_in / second / second}, 6_in,
                                                false, 0.0, 0.0, false);
    }


    /**
     * Command to get rings out of the corner
     *
     * @return Command that removes rings from the corner
     */
    static Command *descoreCorner() {
        auto getOneRingOut = oneRingOutOfCorner();

        return new Sequence({getOneRingOut, getOneRingOut, getOneRingOut, getOneRingOut});
    }

    static Command *oneRingOutOfCorner() {
        return new Sequence({
            (new TankMotionProfiling(drivetrainSubsystem, {20_in / second, 120_in / second / second}, -8_in, false, 0.0,
                                    0.0, false))->with(intakeWithEject),
            (new TankMotionProfiling(drivetrainSubsystem, {25_in / second, 120_in / second / second}, 9_in, false, 0.0,
                                    0.0, false))->with(intakeWithEject),
            drivetrainSubsystem->pct(0.2, 0.2)->with(bottomOuttakeWithEject),
        });
    }
};
