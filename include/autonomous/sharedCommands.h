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
            ->until([]() { return liftSubsystem->getPosition() > 165_deg; })
            ->withTimeout(750_ms)->race(TopIntakePositionCommand::fromForwardPositionCommand(topIntakeSubsystem, 1.2, 0.0));
    }

    static Command *scoreWallStakesWithoutTopIntake() {
        return liftSubsystem->pctCommand(1.0)
            ->until([]() { return liftSubsystem->getPosition() > 165_deg; })
            ->withTimeout(750_ms);
    }

    static Command *driveToAlliance() {
        return new TankMotionProfiling(drivetrainSubsystem, {20_in / second, 100_in / second / second}, 3.0_in, false, 0.0,
                                       0.0, false);
    }


    /**
     * Command to get rings out of the corner
     *
     * @return Command that removes rings from the corner
     */
    static Command *descoreCorner() {
        return new Sequence(
            {drivetrainSubsystem->pct(0.3, 0.3)->with(new ScheduleCommand(bottomOuttakeWithEject))->withTimeout(100_ms), drivetrainSubsystem->pct(1.0, 1.0)->with(new ScheduleCommand(bottomOuttakeWithEject))->withTimeout(150_ms),
             oneRingOutOfCorner()});
    }

    static Command *descoreCornerFull() {
        return new Sequence(
            {drivetrainSubsystem->pct(0.3, 0.3)->with(new ScheduleCommand(bottomOuttakeWithEject))->withTimeout(100_ms), drivetrainSubsystem->pct(1.0, 1.0)->with(new ScheduleCommand(bottomOuttakeWithEject))->withTimeout(150_ms),
             oneRingOutOfCorner(), cycleCorner(), oneRingOutOfCorner(), cycleCorner(), oneRingOutOfCorner(), cycleCorner(), oneRingOutOfCorner()});
    }

    static Command *cycleCorner() {
        return new Sequence({
            drivetrainSubsystem->pct(0.0, 0.0)->withTimeout(300_ms),
            drivetrainSubsystem->pct(0.5, 0.5)->withTimeout(200_ms),
        });
    }

    static Command *oneRingOutOfCorner() {
        return new Sequence({
            drivetrainSubsystem->pct(0.5, 0.5)->with(new ScheduleCommand(cornerClearIntakeSequence))->withTimeout(250_ms),
            drivetrainSubsystem->pct(-0.25, -0.25)->with(new ScheduleCommand(bottomIntakeSubsystem->pctCommand(1.0)->with(TopIntakePositionCommand::fromClosePositionCommand(topIntakeSubsystem, 0.95, 0.0))))->withTimeout(500_ms),
        });
    }
};
