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
                                     lift->moveToPosition(8_deg, 0.3_deg),
                                     topIntake->movePct(0.0),
                                     hook->positionCommand(5_deg),
                             }),
                             new ParallelRaceGroup({
                                     bottomIntake->movePct(0.0),
                                     lift->positionCommand(8_deg),
                                     topIntake->movePct(1.0),
                                     hook->positionCommand(5_deg),
                                     new WaitCommand(70_ms),
                             }),
                             (new ParallelCommandGroup({
                                      bottomIntake->movePct(0.0),
                                      lift->positionCommand(0_deg),
                                      topIntake->movePct(1.0),
                                      hook->positionCommand(0_deg),
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
                                     lift->moveToPosition(8_deg, 0.3_deg),
                                     topIntake->movePct(0.0),
                                     hook->positionCommand(5_deg),
                             }),
                             (new ParallelCommandGroup({
                                      bottomIntake->movePct(0.0),
                                      lift->positionCommand(0_deg),
                                      topIntake->movePct(1.0),
                                      hook->positionCommand(0_deg),
                              }))
                                     ->withTimeout(500_ms)});
    }

    /**
     * Command to get rings out of the corner
     *
     * @return Command that removes rings from the corner
     */
    static Command *descoreCorner() {
        Angle startAngle = drivetrain->getAngle();
        return new Sequence({
                new InstantCommand([&startAngle]() { startAngle = drivetrain->getAngle(); }, {}),
                new ScheduleCommand(hook->positionCommand(0.58)),
                drivetrain->pct(0.4, 0.4)->withTimeout(0.5_s),
                new ScheduleCommand(intakeOntoGoal),
                drivetrain->pct(-0.18, 0.18)->withTimeout(0.5_s),
                drivetrain->pct(-0.4, 0.25)->until([&startAngle]() {
                    return Qabs(angleDifference(drivetrain->getAngle(), startAngle)) > 80_deg;
                }),
                new ScheduleCommand(hook->positionCommand(0.0)),
                drivetrain->pct(-0.3, -0.3)->withTimeout(0.5_s),
                drivetrain->pct(0.3, 0.3)->withTimeout(2_s),
        });
    }
};
