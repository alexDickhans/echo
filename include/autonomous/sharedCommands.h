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
        return liftSubsystem->positionCommand(180_deg, 20_deg)->withTimeout(600_ms);
    }

    static Command *scoreWallStakes() {
        return liftSubsystem->pctCommand(1.0)->until([]() { return liftSubsystem->getPosition() > 150_deg; })->
                withTimeout(550_ms);
    }

    static Command *arcOntoAlliance(bool flip, bool pos) {
        return new TankMotionProfiling(drivetrainSubsystem, {20_in / second, 70_in / second / second}, 2_in, flip,
                                        90_deg, (pos ? -1.0 : 1.0) * 72_deg / 2_in);
    }


    /**
     * Command to get rings out of the corner
     *
     * @return Command that removes rings from the corner
     */
    static Command *descoreCorner() {
        return new Sequence({
            new TankMotionProfiling(drivetrainSubsystem, {20_in/second, 50_in/second/second}, -14_in, false, 0.0, 0.0, false),
            new ScheduleCommand(doinker->levelCommand(true)),
            (new Rotate(drivetrainSubsystem, drivetrainSubsystem->getAngle() - 8_deg, false))->withTimeout(400_ms),
            new TankMotionProfiling(drivetrainSubsystem, {20_in/second, 50_in/second/second}, -3_in, false, 0.0, 0.0, false),
            (new Rotate(drivetrainSubsystem, drivetrainSubsystem->getAngle() - 8_deg, false))->withTimeout(400_ms),
            new TankMotionProfiling(drivetrainSubsystem, {25_in/second, 80_in/second/second}, 15_in, false, 0.0, -95_deg/15_in, false),
            new ScheduleCommand(doinker->levelCommand(false)),
        });
    }
};
