#pragma once

#include <cassert>
#include "bezier/bezier.h"
#include "combinedMotionProfile.h"
#include "config.h"
#include "json/asset.hpp"
#include "json/json.hpp"
#include "motionProfile.h"
#include "path.h"
#include "utils/motor.h"

class BezierMotionProfile : public MotionProfile {
private:
    std::vector<Bezier> beziers;
    std::vector<double> time, velocity, t;
    bool reversed{false};

    static double limitSpeed(const QCurvature curvature) {
        if (curvature.getValue() == 0.0)
            return 1.0;

        return 1.0 / (1.0 + abs(curvature.getValue() * 0.5) * CONFIG::TRACK_WIDTH.getValue());
    }

    static std::vector<QVelocity> limitedSpeed(const std::vector<QLength> &distance,
                                               const std::vector<QVelocity> &velocity,
                                               const std::vector<QAcceleration> &accel) {
        std::vector<QVelocity> result;

        result.emplace_back(velocity[0]);

        for (size_t i = 1; i < velocity.size(); i++) {
            QLength deltaDistance = Qabs(distance[i] - distance[i - 1]);

            result.emplace_back(
                    std::min(Qsqrt(MOTOR11W::torquePercent((Qabs(result[i - 1]) / CONFIG::MAX_SPEED).getValue()) *
                                           Qsq(result[i - 1]) +
                                   2.0 * accel[i] * deltaDistance),
                             velocity[i]));
        }

        return result;
    }

public:
    explicit BezierMotionProfile(std::vector<Bezier> beziers, QVelocity startSpeed, QVelocity endSpeed) :
        beziers(std::move(beziers)) {

        assert(!this->beziers.empty());
        reversed = this->beziers.at(0).getReversed();
        for (const auto &bezier: this->beziers) {
            assert(reversed == bezier.getReversed());
        }

        calculate(startSpeed, endSpeed);
    }

    static CombinedMotionProfile build(const asset path, const bool mirror) {
        const Json parsed_path = open_asset_as_json(path);

        CombinedMotionProfile motionProfiles{{}};

        QVelocity startSpeed = parsed_path["start_speed"].number_value();
        QVelocity endSpeed = parsed_path["end_speed"].number_value();

        std::vector<Bezier> segments{Bezier(parsed_path["segments"][0], mirror)};

        bool last_inverted = segments[0].getReversed();

        for (size_t i = 1; i < parsed_path["segments"].array_items().size(); i++) {
            Bezier segment(parsed_path["segments"][i], mirror);
            if (segment.getReversed() ^ last_inverted || segment.getStopBegin()) {
                if (!segments.empty()) {
                    last_inverted = segment.getReversed();
                    motionProfiles.addMP(new BezierMotionProfile(segments, startSpeed, 0.0));
                    startSpeed = 0.0;
                    segments.clear();
                }
            }
            segments.emplace_back(segment);
        }

        motionProfiles.addMP(new BezierMotionProfile(segments, startSpeed, endSpeed));

        for (const auto &command: parsed_path["commands"].array_items()) {
            motionProfiles.addCommand(command["t"].number_value(), command["name"].string_value());
        }

        return motionProfiles;
    }

    void calculate(const QVelocity startSpeed, const QVelocity endSpeed) {
        std::vector<QLength> distance{0.0};
        std::vector<QVelocity> velocity{
                std::min(limitSpeed(beziers[0].getCurvature(0.0)) * CONFIG::MAX_SPEED, startSpeed)};
        std::vector<QAcceleration> accel{0.0};

        QLength accumulatedDistance = 0.0;

        this->t.emplace_back(0.0);

        for (size_t i = 0; i < beziers.size(); i++) {
            auto length = beziers[i].getDistance();
            const auto count = std::max(5, static_cast<int>(std::ceil((length / 2_in).getValue())));

            for (size_t j = 1; j <= count; j++) {
                const auto t = static_cast<double>(j) / static_cast<double>(count);

                this->t.emplace_back(static_cast<double>(i) + t);

                const auto curvature = beziers[i].getCurvature(t);

                distance.emplace_back(accumulatedDistance + beziers[i].getDistanceAtT(t));
                velocity.emplace_back(std::min(limitSpeed(curvature) * CONFIG::MAX_SPEED, beziers[i].velocity));
                accel.emplace_back(beziers[i].acceleration);
            }

            accumulatedDistance += beziers[i].getDistance();
        }

        velocity[velocity.size() - 1] = std::min(endSpeed, velocity[velocity.size() - 1]);

        auto leftPass = limitedSpeed(distance, velocity, accel);

        std::ranges::reverse(distance);
        std::ranges::reverse(velocity);
        std::ranges::reverse(accel);

        auto rightPass = limitedSpeed(distance, velocity, accel);

        std::ranges::reverse(distance);
        std::ranges::reverse(velocity);
        std::ranges::reverse(rightPass);

        for (size_t i = 0; i < velocity.size(); i++) {
            velocity[i] = std::min(leftPass[i], rightPass[i]);
        }

        QTime time = 0.0;

        this->time.emplace_back(time.getValue());
        this->velocity.emplace_back(velocity[0].getValue());

        for (size_t i = 1; i < velocity.size(); i++) {
            // add stuff to current time
            auto delta_distance = distance[i] - distance[i - 1];
            auto change_v = Qsq(velocity[i]) - Qsq(velocity[i - 1]);
            auto a = change_v / (2.0 * delta_distance);

            if (Qabs(a).getValue() > 0.1) {
                time += (velocity[i] - velocity[i - 1]) / a;
            } else {
                time += delta_distance / velocity[i];
            }

            this->time.emplace_back(time.getValue());
            this->velocity.emplace_back(velocity[i].getValue());
        }
    }

    std::optional<MotionCommand> get(const QTime time) override {
        const auto t = std::clamp(interp(this->time, this->t, time.getValue()), 0.0,
                                  static_cast<double>(this->beziers.size()));
        auto i = std::clamp(static_cast<size_t>(t), (size_t) 0, this->beziers.size() - 1);
        auto tLocal = fmod(t, 1.0000001);

        auto invertedMultiplier = this->reversed ? -1.0 : 1.0;

        auto desiredVelocity = interp(this->time, this->velocity, time.getValue()) * metre / second;
        auto curvature = beziers[i].getCurvature(tLocal);

        auto pose = beziers[i].get(tLocal);

        pose.z() += this->reversed ? M_PI : 0.0;

        return MotionCommand{pose, invertedMultiplier * desiredVelocity, desiredVelocity * curvature, t};
    }

    [[nodiscard]] QTime getDuration() const override { return this->time[this->time.size() - 1]; }

    double maxT() override { return beziers.size(); }

    ~BezierMotionProfile() override = default;
};

#define BEZIER_MP_ASSET(x)                                                                                             \
    ASSET(x##_json);                                                                                                   \
    auto x = BezierMotionProfile::build(x##_json, false);
#define BEZIER_MIRRORED_MP_ASSET(x)                                                                                    \
    ASSET(x##_json);                                                                                                   \
    auto x##_red = BezierMotionProfile::build(x##_json, false);                                                        \
    auto x##_blue = BezierMotionProfile::build(x##_json, true);
