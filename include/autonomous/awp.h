#pragma once

class AWP {
public:
    static Command *awp(const bool flip) {
        return new Sequence({
                drivetrain->setNorm(Eigen::Vector2f(0.0, (64_in).getValue()), Eigen::Matrix2f::Identity() * 0.2,
                                    -90_deg, flip),
                new ScheduleCommand(topIntake->movePct(1.0)),
                (new Rotate(drivetrain, -90_deg, flip, -2000, false))->withTimeout(0.5_s),
                new TankMotionProfiling(drivetrain, {65_in / second, 100_in / second / second}, 16_in, flip, -90_deg,
                                        0.0),
                (new Rotate(drivetrain, 180_deg, false, 0.0))->withTimeout(0.8_s),
                new ScheduleCommand(goalClamp->levelCommand(false)),
                (new DriveToGoal(drivetrain, CONFIG::GOAL_PID, -0.7))
                        ->until([&]() { return goalClampDistanceSensor.get_distance() < 25; })
                        ->withTimeout(1.5_s),
        });
    }
};
