#pragma once

#include "subsystems/subsystems.h"

BEZIER_MP_ASSET(skills_1);
BEZIER_MP_ASSET(skills_2);

class Skills {
public:
  static Command *skills() {
    return new Sequence({
            drivetrain->setNorm(Eigen::Vector2f(0.0, (64_in).getValue()), Eigen::Matrix2f::Identity() * 0.2, -90_deg,
                                false),
            new ScheduleCommand(topIntake->movePct(1.0)),
            (new Rotate(drivetrain, -90_deg, false, -2000, false))->withTimeout(0.5_s),
            new TankMotionProfiling(drivetrain, {65_in / second, 100_in / second / second}, 16_in, false, -90_deg, 0.0),
            (new Rotate(drivetrain, 180_deg, false, 0.0))->withTimeout(0.8_s),
            new ScheduleCommand(goalClamp->levelCommand(false)),
            (new DriveToGoal(drivetrain, CONFIG::GOAL_PID, -0.7))
                    ->until([&]() { return goalClampDistanceSensor.get_distance() < 25; })
                    ->withTimeout(1.5_s),
            new ScheduleCommand(goalClampTrue),
            new Ramsete(drivetrain, 0.6, 25.0, &skills_1),
            drivetrain->pct(0.5, 0.5)->race((new Sequence({new ParallelRaceGroup({
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
            new TankMotionProfiling(drivetrain, {65_in / second, 100_in / second / second}, -18_in, false, 0_deg, 0.0),
            (new Rotate(drivetrain, -90_deg, false, 0.0))->withTimeout(0.8_s),
            new Ramsete(drivetrain, 0.6, 25.0, &skills_2),
    });
  }
};

