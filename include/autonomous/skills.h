#pragma once

#include "sharedCommands.h"
#include "subsystems/subsystems.h"

BEZIER_MP_ASSET(skills_1);
BEZIER_MP_ASSET(skills_1_1);
BEZIER_MP_ASSET(skills_2);
BEZIER_MP_ASSET(skills_3);
BEZIER_MP_ASSET(skills_4);
BEZIER_MP_ASSET(skills_5);

class Skills {
public:
    static Command *skills() {
        return new Sequence({
                drivetrain->setNorm(Eigen::Vector2f(0.0, (64_in).getValue()), Eigen::Matrix2f::Identity() * 0.05,
                                    -90_deg, false),
                new ScheduleCommand(SharedCommands::scoreAlliance()),
                (new Rotate(drivetrain, -90_deg, false, -0.5, false))->withTimeout(0.5_s),
                new TankMotionProfiling(drivetrain, {65_in / second, 150_in / second / second}, 13_in, false, -90_deg,
                                        0.0),
                (new Rotate(drivetrain, 180_deg, false, 0.0))->withTimeout(0.6_s),
                new ScheduleCommand(goalClamp->levelCommand(false)),
                new TankMotionProfiling(drivetrain, {35_in / second, 100_in / second / second}, -12_in, false, 180_deg,
                                        0.0, true, 0.0, -10_in / second),
                new TankMotionProfiling(drivetrain, {10_in / second, 150_in / second / second}, -5_in, false, 180_deg,
                                        0.0, true, -10_in / second, 0_in / second),
                new ScheduleCommand(goalClampTrue),
                new Ramsete(drivetrain, &skills_1),
                drivetrain->pct(0.15, 0.15)
                        ->race((new Sequence({new ParallelRaceGroup({
                                                      bottomIntake->movePct(0.0),
                                                      lift->moveToPosition(33_deg, 0.3_deg),
                                                      topIntake->movePct(0.0),
                                                      hook->positionCommand(5_deg),
                                              }),
                                              new ParallelRaceGroup({
                                                      bottomIntake->movePct(0.0),
                                                      lift->moveToPosition(33_deg, 1_deg),
                                                      topIntake->movePct(0.0),
                                                      hook->positionCommand(0_deg),
                                              }),
                                              (new ParallelCommandGroup({
                                                       bottomIntake->movePct(0.0),
                                                       lift->positionCommand(33_deg),
                                                       topIntake->movePct(-1.0),
                                                       hook->positionCommand(0_deg),
                                               }))
                                                      ->withTimeout(0.8_s)}))
                                       ->asProxy()),
                new TankMotionProfiling(drivetrain, {65_in / second, 100_in / second / second}, -18_in, false, 0_deg,
                                        0.0, true),
                (new Rotate(drivetrain, -90_deg, false, 0.0))->withTimeout(0.8_s),
                new Ramsete(drivetrain, &skills_1_1),
                drivetrain->pct(0.15, 0.15)
                        ->race((new Sequence({new ParallelRaceGroup({
                                                      bottomIntake->movePct(0.0),
                                                      lift->moveToPosition(33_deg, 0.3_deg),
                                                      topIntake->movePct(0.0),
                                                      hook->positionCommand(5_deg),
                                              }),
                                              new ParallelRaceGroup({
                                                      bottomIntake->movePct(0.0),
                                                      lift->moveToPosition(33_deg, 1_deg),
                                                      topIntake->movePct(0.0),
                                                      hook->positionCommand(0_deg),
                                              }),
                                              (new ParallelCommandGroup({
                                                       bottomIntake->movePct(0.0),
                                                       lift->positionCommand(33_deg),
                                                       topIntake->movePct(-1.0),
                                                       hook->positionCommand(0_deg),
                                               }))
                                                      ->withTimeout(0.8_s)}))
                                       ->asProxy()),
                new TankMotionProfiling(drivetrain, {65_in / second, 100_in / second / second}, -18_in, false, 0_deg,
                                        0.0, true),
                (new Rotate(drivetrain, -90_deg, false, 0.0))->withTimeout(0.8_s),
                new Ramsete(drivetrain, &skills_2),
                new ScheduleCommand(SharedCommands::scoreAlliance()),
                (new Rotate(drivetrain, 90_deg, false, -2000, false))->withTimeout(0.8_s),
                new TankMotionProfiling(drivetrain, {65_in / second, 150_in / second / second}, 17_in, false, 90_deg,
                                        0.0, true),
                (new Rotate(drivetrain, -20_deg, false, 0, true))->withTimeout(0.8_s),
                new TankMotionProfiling(drivetrain, {25_in / second, 60_in / second / second}, -17_in, false, -20_deg,
                                        0.0, true),
                new ScheduleCommand(goalClampTrue),
                new Ramsete(drivetrain, &skills_3),
                new TankMotionProfiling(drivetrain, {65_in / second, 100_in / second / second}, 16_in, false, -45_deg,
                                        0.0, true),
                (new Rotate(drivetrain, 180_deg, false, 0, true))->withTimeout(1.0_s),
                new TankMotionProfiling(drivetrain, {35_in / second, 100_in / second / second}, -15_in, false, 180_deg,
                                        0.0, true, 0.0, -20_in / second),
                new TankMotionProfiling(drivetrain, {10_in / second, 80_in / second / second}, -6_in, false, 180_deg,
                                        0.0, true, -20_in / second, 0_in / second),
                new ScheduleCommand(goalClampTrue),
                new Ramsete(drivetrain, &skills_4),
                drivetrain->pct(0.0, 0.0)->withTimeout(0.8_s),
                drivetrain->pct(0.15, 0.15)
                        ->race((new Sequence({new ParallelRaceGroup({
                                                      bottomIntake->movePct(0.0),
                                                      lift->moveToPosition(33_deg, 0.3_deg),
                                                      topIntake->movePct(0.0),
                                                      hook->positionCommand(5_deg),
                                              }),
                                              new ParallelRaceGroup({
                                                      bottomIntake->movePct(0.0),
                                                      lift->moveToPosition(33_deg, 1_deg),
                                                      topIntake->movePct(0.0),
                                                      hook->positionCommand(0_deg),
                                              }),
                                              (new ParallelCommandGroup({
                                                       bottomIntake->movePct(0.0),
                                                       lift->positionCommand(33_deg),
                                                       topIntake->movePct(-1.0),
                                                       hook->positionCommand(0_deg),
                                               }))
                                                      ->withTimeout(0.8_s)}))
                                       ->asProxy()),
                new Ramsete(drivetrain, &skills_5),
        });
    }
};
