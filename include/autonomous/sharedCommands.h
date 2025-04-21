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
        return new TankMotionProfiling(drivetrainSubsystem, {10_in / second, 60_in / second / second}, 2_in, false, 0.0,
                                       0.0, false);
    }


    /**
     * Command to get rings out of the corner
     *
     * @return Command that removes rings from the corner
     */
    static Command *descoreCorner() {
        return new Sequence(
            {drivetrainSubsystem->pct(0.4, 0.4)->race(bottomOuttakeWithEject->asProxy())->withTimeout(200_ms),
             oneRingOutOfCorner(), cycleCorner(), oneRingOutOfCorner(), cycleCorner(), oneRingOutOfCorner()});
    }

    static Command *cycleCorner() {
        return new Sequence({
            drivetrainSubsystem->pct(0.0, 0.0)->race(intakeWithEject->asProxy())->withTimeout(300_ms),
            drivetrainSubsystem->pct(0.4, 0.4)->race(intakeWithEject->asProxy())->withTimeout(100_ms),
        });
    }

    static Command *oneRingOutOfCorner() {
        return new Sequence({
            drivetrainSubsystem->pct(0.2, 0.2)->race(bottomOuttakeWithEject->asProxy())->withTimeout(300_ms),
            drivetrainSubsystem->pct(0.2, 0.2)->race(intakeWithEject->asProxy())->withTimeout(300_ms),
            drivetrainSubsystem->pct(-0.2, -0.2)->race(intakeWithEject->asProxy())->withTimeout(400_ms),
        });
    }
};
