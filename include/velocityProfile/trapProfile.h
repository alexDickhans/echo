#pragma once

class TrapProfile {
public:
    class Constraints {
    public:
        double maxVelocity;
        double maxAcceleration;

        Constraints(const double max_velocity, const double max_acceleration) :
            maxVelocity(max_velocity), maxAcceleration(max_acceleration) {}
    };

    class State {
    public:
        double position;
        double velocity;

        State(double position, double velocity) : position(position), velocity(velocity) {}
    };

private:
    static bool shouldFlipAcceleration(State initial, State goal) { return initial.position > goal.position; }
    Constraints constraints;

    static State direct(const State &in, const double direction) {
        State result = State(in.position, in.velocity);
        result.position = result.position * direction;
        result.velocity = result.velocity * direction;
        return result;
    }

public:
    explicit TrapProfile(const Constraints &constraints) : constraints(constraints) {}

    [[nodiscard]] State calculate(const QTime t, const State &initial, const State &end) const {
        const double direction = shouldFlipAcceleration(initial, end) ? -1.0 : 1.0;
        auto current = direct(initial, direction);
        auto goal = direct(end, direction);

        if (current.velocity > constraints.maxVelocity) {
            current.velocity = constraints.maxVelocity;
        }

        double cutoffBegin = current.velocity / constraints.maxAcceleration;
        double cutoffDistBegin = cutoffBegin * cutoffBegin * constraints.maxAcceleration / 2.0;

        double cutoffEnd = goal.velocity / constraints.maxAcceleration;
        double cutoffDistEnd = cutoffEnd * cutoffEnd * constraints.maxAcceleration / 2.0;

        double fullTrapezoidDist = cutoffDistBegin + (goal.position - current.position) + cutoffDistEnd;
        double accelerationTime = constraints.maxVelocity / constraints.maxAcceleration;

        double fullSpeedDist = fullTrapezoidDist - accelerationTime * accelerationTime * constraints.maxAcceleration;

        // Handle the case where the profile never reaches full speed
        if (fullSpeedDist < 0) {
            accelerationTime = sqrt(fullTrapezoidDist / constraints.maxAcceleration);
            fullSpeedDist = 0;
        }

        double endAccel = accelerationTime - cutoffBegin;
        double endFullspeed = endAccel + fullSpeedDist / constraints.maxVelocity;
        double endDecel = endFullspeed + accelerationTime - cutoffEnd;
        State result = State(current.position, current.velocity);

        if (t.getValue() < endAccel) {
            result.velocity += t.getValue() * constraints.maxAcceleration;
            result.position += (current.velocity + t.getValue() * constraints.maxAcceleration / 2.0) * t.getValue();
        } else if (t.getValue() < endFullspeed) {
            result.velocity = constraints.maxVelocity;
            result.position += (current.velocity + endAccel * constraints.maxAcceleration / 2.0) * endAccel +
                               constraints.maxVelocity * (t.getValue() - endAccel);
        } else if (t.getValue() <= endDecel) {
            result.velocity = goal.velocity + (endDecel - t.getValue()) * constraints.maxAcceleration;
            double timeLeft = endDecel - t.getValue();
            result.position =
                    goal.position - (goal.velocity + timeLeft * constraints.maxAcceleration / 2.0) * timeLeft;
        } else {
            result = goal;
        }

        return direct(result, direction);
    }
};
