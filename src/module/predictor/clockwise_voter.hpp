#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numbers>
#include <optional>

#include "utility/clock.hpp"
#include "utility/math/angle.hpp"
#include "utility/math/conversion.hpp"
#include "utility/time.hpp"

namespace rmcs::predictor {

struct PowerRuneClockwiseVoter {
    static constexpr auto kScoreLimit      = 12;
    static constexpr auto kBootstrapScore  = 4;
    static constexpr auto kDeltaLowerBound = util::deg2rad(0.6);
    static constexpr auto kDeltaUpperBound = util::deg2rad(55.0);
    static constexpr auto kNominalFrameDt  = 0.01;
    static constexpr auto kUpperBoundEps   = 1e-3;

    int clockwise { 1 };
    int score { kBootstrapScore };
    std::optional<double> last_blade_angle;

    static constexpr auto normalize_clockwise(int clockwise) -> int {
        return clockwise >= 0 ? 1 : -1;
    }

    auto reset(int seed_clockwise, std::optional<double> seed_blade_angle = std::nullopt) -> void {
        clockwise        = normalize_clockwise(seed_clockwise);
        score            = clockwise * kBootstrapScore;
        last_blade_angle = seed_blade_angle;
    }

    static auto adaptive_upper_bound(double dt_s) -> double {
        if (!(dt_s > 0.0)) {
            return kDeltaUpperBound;
        }
        const auto scale = std::max(1.0, dt_s / kNominalFrameDt);
        return std::clamp(
            kDeltaUpperBound * scale, kDeltaUpperBound, std::numbers::pi - kUpperBoundEps);
    }

    auto update(double observed_blade_angle, double dt_s = 0.0) -> int {
        observed_blade_angle = util::normalize_angle(observed_blade_angle);
        if (last_blade_angle.has_value()) {
            const auto delta     = util::normalize_angle(observed_blade_angle - *last_blade_angle);
            const auto abs_delta = std::abs(delta);
            const auto upper_bound = adaptive_upper_bound(dt_s);
            if (abs_delta >= kDeltaLowerBound && abs_delta <= upper_bound) {
                score += (delta >= 0.0) ? 1 : -1;
                score = std::clamp(score, -kScoreLimit, kScoreLimit);
                if (score > 0) clockwise = 1;
                if (score < 0) clockwise = -1;
            }
        }
        last_blade_angle = observed_blade_angle;
        return clockwise;
    }
};

struct PowerRuneClockwiseTracker {
    using Clock = util::Clock;

    int clockwise { 1 };
    PowerRuneClockwiseVoter voter {};
    std::optional<Clock::time_point> last_observe_stamp;

    auto reset(int seed_clockwise, std::optional<double> seed_blade_angle = std::nullopt,
        std::optional<Clock::time_point> seed_stamp = std::nullopt) -> int {
        clockwise = PowerRuneClockwiseVoter::normalize_clockwise(seed_clockwise);
        voter.reset(clockwise, seed_blade_angle);
        last_observe_stamp = seed_stamp;
        return clockwise;
    }

    auto update(double observed_blade_angle, Clock::time_point stamp) -> int {
        auto dt_s = 0.0;
        if (last_observe_stamp.has_value()) {
            const auto dt = util::delta_time(stamp, *last_observe_stamp);
            if (dt > std::chrono::duration<double>::zero()) {
                dt_s = dt.count();
            }
        }

        clockwise         = voter.update(observed_blade_angle, dt_s);
        last_observe_stamp = stamp;
        return clockwise;
    }
};

} // namespace rmcs::predictor
