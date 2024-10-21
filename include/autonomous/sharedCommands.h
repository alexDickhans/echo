#pragma once

class SharedCommands {
public:
    static Command *scoreAlliance() {
        return new Sequence({new ParallelRaceGroup({
                                     bottomIntake->movePct(0.0),
                                     lift->moveToPosition(7_deg, 0.3_deg),
                                     topIntake->movePct(0.0),
                                     hook->positionCommand(5_deg),
                             }),
                             new ParallelRaceGroup({
                                     bottomIntake->movePct(0.0),
                                     lift->positionCommand(7_deg),
                                     topIntake->movePct(1.0),
                                     hook->positionCommand(5_deg),
                                     new WaitCommand(100_ms),
                             }),
                             (new ParallelCommandGroup({
                                      bottomIntake->movePct(0.0),
                                      lift->positionCommand(0_deg),
                                      topIntake->movePct(1.0),
                                      hook->positionCommand(0_deg),
                              }))
                                     ->withTimeout(500_ms)});
    }


    static Command *scoreAlliance2() {
        return new Sequence({new ParallelRaceGroup({
                                     bottomIntake->movePct(0.0),
                                     lift->moveToPosition(7_deg, 0.3_deg),
                                     topIntake->movePct(0.0),
                                     hook->positionCommand(5_deg),
                             }),
                             new ParallelRaceGroup({
                                     bottomIntake->movePct(0.0),
                                     lift->positionCommand(7_deg),
                                     topIntake->movePct(1.0),
                                     hook->positionCommand(5_deg),
                                     new WaitCommand(10_ms),
                             }),
                             (new ParallelCommandGroup({
                                      bottomIntake->movePct(0.0),
                                      lift->positionCommand(0_deg),
                                      topIntake->movePct(1.0),
                                      hook->positionCommand(0_deg),
                              }))
                                     ->withTimeout(500_ms)});
    }

    static void updateAllianceColor() {
        // TODO
    }
};
