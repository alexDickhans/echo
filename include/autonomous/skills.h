#pragma once

#include "sharedCommands.h"
#include "subsystems/subsystems.h"

BEZIER_MP_ASSET(skills_1);
BEZIER_MP_ASSET(skills_2);
BEZIER_MP_ASSET(skills_3);
BEZIER_MP_ASSET(skills_4);
BEZIER_MP_ASSET(skills_5);

class Skills {
public:
    static Command *skills() {
        return new Sequence({drivetrain->setNorm(Eigen::Vector2f(0.0, (64_in).getValue()),
                                                 Eigen::Matrix2f::Identity() * 0.05, -90_deg, false),
                             new ScheduleCommand(SharedCommands::scoreAlliance()),
                             (new Rotate(drivetrain, -90_deg, false, -0.5, false))->withTimeout(0.5_s),
                             new TankMotionProfiling(drivetrain, {65_in / second, 100_in / second / second}, 17_in,
                                                     false, -90_deg, 0.0),
                             (new Rotate(drivetrain, 180_deg, false, 0.0))->withTimeout(0.7_s),
                             new ScheduleCommand(goalClamp->levelCommand(false)),
                             new TankMotionProfiling(drivetrain, {25_in / second, 50_in / second / second}, -24_in,
                                                     false, 180_deg, 0.0, true, 0.0, 0_in / second),
                             new ScheduleCommand(goalClampTrue),
                             new Ramsete(drivetrain, &skills_1),
                             drivetrain->pct(0.15, 0.15)
                                     ->race((new Sequence({new ParallelRaceGroup({
                                                                   bottomIntake->movePct(0.0),
                                                                   lift->moveToPosition(33_deg, 0.3_deg),
                                                                   topIntake->movePct(0.0),
                                                           }),
                                                           new ParallelRaceGroup({
                                                                   bottomIntake->movePct(0.0),
                                                                   lift->moveToPosition(33_deg, 1_deg),
                                                                   topIntake->movePct(0.0),
                                                           }),
                                                           (new ParallelCommandGroup({
                                                                    bottomIntake->movePct(0.0),
                                                                    lift->positionCommand(33_deg),
                                                                    topIntake->movePct(-1.0),
                                                            }))
                                                                   ->withTimeout(0.8_s)}))
                                                    ->asProxy()),
                             new TankMotionProfiling(drivetrain, {65_in / second, 100_in / second / second}, -18_in,
                                                     false, 0_deg, 0.0, true),
                             (new Rotate(drivetrain, -90_deg, false, 0.0))->withTimeout(0.65_s),
                             new Ramsete(drivetrain, &skills_2),
                             new ScheduleCommand(SharedCommands::scoreAlliance2()),
                             (new Rotate(drivetrain, 90_deg, false, -1500, false))->withTimeout(1.0_s),
                             new TankMotionProfiling(drivetrain, {65_in / second, 100_in / second / second}, 18_in,
                                                     false, 90_deg, 0.0, true),
                             (new Rotate(drivetrain, -20_deg, false, 0, true))->withTimeout(0.8_s),
                             new TankMotionProfiling(drivetrain, {25_in / second, 60_in / second / second}, -25_in,
                                                     false, -20_deg, 0.0, true),
                             new ScheduleCommand(goalClampTrue),
                             new Ramsete(drivetrain, &skills_3),
                             new TankMotionProfiling(drivetrain, {62_in / second, 100_in / second / second}, 17_in,
                                                     false, -45_deg, 0.0, true),
                             (new Rotate(drivetrain, 180_deg, false, 0, true))->withTimeout(1.0_s),
                             new TankMotionProfiling(drivetrain, {28_in / second, 50_in / second / second}, -26_in,
                                                     false, 180_deg, 0.0, true),
                             new ScheduleCommand(goalClampTrue),
                             new Ramsete(drivetrain, &skills_4),
                             drivetrain->pct(0.0, 0.0)->withTimeout(0.5_s),
                             drivetrain->pct(0.15, 0.15)
                                     ->race((new Sequence({new ParallelRaceGroup({
                                                                   bottomIntake->movePct(0.0),
                                                                   lift->moveToPosition(33_deg, 0.3_deg),
                                                                   topIntake->movePct(0.0),
                                                           }),
                                                           new ParallelRaceGroup({
                                                                   bottomIntake->movePct(0.0),
                                                                   lift->moveToPosition(33_deg, 1_deg),
                                                                   topIntake->movePct(0.0),
                                                           }),
                                                           (new ParallelCommandGroup({
                                                                    bottomIntake->movePct(0.0),
                                                                    lift->positionCommand(33_deg),
                                                                    topIntake->movePct(-1.0),
                                                            }))
                                                                   ->withTimeout(0.8_s)}))
                                                    ->asProxy()),
                             new Ramsete(drivetrain, &skills_5),
                             (new ScheduleCommand(hang->levelCommand(true)))
                                     ->with(drivetrain->velocityCommand(37_in / second, 37_in / second)
                                                    ->until([]() { return Qabs(drivetrain->getRoll()) > 8_deg; })
                                                    ->andThen((drivetrain->pct(-1.0, -1.0)
                                                                       ->with(hook->pctCommand(1.0)->asProxy())
                                                                       ->withTimeout(300_ms)
                                                                       ->andThen(drivetrain->pct(1.0, 1.0)
                                                                                         ->with(hook->pctCommand(-1.0)->asProxy())
                                                                                         ->withTimeout(300_ms)))
                                                                      ->repeatedly()
                                                                      ->until([]() {
                                                                          return Qabs(drivetrain->getRoll()) < 5_deg;
                                                                      })))

        });
    }
};
