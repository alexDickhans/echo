#pragma once

#include "sharedCommands.h"
#include "subsystems/subsystems.h"

BEZIER_MP_ASSET(skills_1);
BEZIER_MP_ASSET(skills_2);
BEZIER_MP_ASSET(skills_3);

class Skills {
public:
    static Command *skills() {
        return new Sequence({
                drivetrain->setNorm(Eigen::Vector2f(0.0, (64_in).getValue()), Eigen::Matrix2f::Identity() * 0.05,
                                    -90_deg, false),
                new ScheduleCommand(SharedCommands::scoreAlliance()),
                (new Rotate(drivetrain, -90_deg, false, -2000, false))->withTimeout(0.5_s),
                new TankMotionProfiling(drivetrain, {65_in / second, 100_in / second / second}, 16_in, false, -90_deg,
                                        0.0),
                (new Rotate(drivetrain, 180_deg, false, 0.0))->withTimeout(0.8_s),
                new ScheduleCommand(goalClamp->levelCommand(false)),
                new TankMotionProfiling(drivetrain, {35_in / second, 70_in / second / second}, -18_in, false, 180_deg,
                                        0.0, true, 0.0, 0_in/second),
                new ScheduleCommand(goalClampTrue),
                new Ramsete(drivetrain, &skills_1),
                drivetrain->pct(0.15, 0.15)->race((new Sequence({new ParallelRaceGroup({
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
                drivetrain->setNorm(Eigen::Vector2f(0.0, (64_in).getValue()), Eigen::Matrix2f::Identity() * 0.2,
                                    -90_deg, false),
                new ScheduleCommand(SharedCommands::scoreAlliance()),
          (new Rotate(drivetrain, 90_deg, false, -2000, false))->withTimeout(0.5_s),
          new TankMotionProfiling(drivetrain, {65_in / second, 100_in / second / second}, 18_in, false, 90_deg,
          0.0, true),
          (new Rotate(drivetrain, -30_deg, false, 0, true))->withTimeout(0.8_s),
new TankMotionProfiling(drivetrain, {25_in / second, 60_in / second / second}, -18_in, false, 90_deg,
          0.0, true),
                new Ramsete(drivetrain, &skills_3),

        });
    }
};
