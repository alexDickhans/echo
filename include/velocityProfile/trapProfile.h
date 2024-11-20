#pragma once

#include "units/units.hpp"

class TrapProfile {
public:
    class Constraints {
    public:
        double maxVelocity;
        double maxAcceleration;

        Constraints(const double max_velocity, const double max_acceleration);
    };

    class State {
    public:
        double position;
        double velocity;

        State(double position, double velocity);
    };

private:
    static bool shouldFlipAcceleration(State initial, State goal);
    Constraints constraints;

    static State direct(const State &in, const double direction);

public:
    explicit TrapProfile(const Constraints &constraints);

    [[nodiscard]] State calculate(const QTime t, const State &initial, const State &end) const;
};
