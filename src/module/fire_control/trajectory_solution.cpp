#include "trajectory_solution.hpp"

#include <cmath>

using namespace rmcs::fire_control;

auto TrajectorySolution::solve() const -> std::optional<Output> {
    auto const target_d = std::hypot(input.target_position.x(), input.target_position.y());
    auto const target_h = input.target_position.z();

    if (input.v0 <= 0 || target_d <= 0) return std::nullopt;

    double pitch   = std::atan2(target_h, target_d);
    auto const yaw = std::atan2(input.target_position.y(), input.target_position.x());

    for (int i = 0; i < kMaxIterateCount; ++i) {
        auto [actual_h, t] = Estimate(input.v0, pitch, target_d, kAirResistanceCoefficient);

        auto h_error = target_h - actual_h;
        if (std::abs(h_error) < kHeightErrorThreold) {
            auto result     = Output {};
            result.fly_time = t;
            result.yaw      = yaw;
            result.pitch    = pitch;
            return result;
        }

        pitch += std::atan2(h_error, target_d);

        if (std::abs(pitch) > kMaxPitchThreold) break;
    }

    return std::nullopt;
}

auto TrajectorySolution::Estimate(double v0, double pitch, double d, double air_resistance) const
    -> std::tuple<double, double> {
    double x = 0, y = 0, t = 0;
    double vx = v0 * std::cos(pitch);
    double vy = v0 * std::sin(pitch);

    double prev_x = 0, prev_y = 0, prev_t = 0;

    while (x < d) {
        prev_x = x;
        prev_y = y;
        prev_t = t;

        const double v = std::sqrt(vx * vx + vy * vy);

        vx -= air_resistance * v * vx * kEstimateDeltaTime;
        vy -= (kGravity + air_resistance * v * vy) * kEstimateDeltaTime;

        x += vx * kEstimateDeltaTime;
        y += vy * kEstimateDeltaTime;
        t += kEstimateDeltaTime;

        if (t > kEstimateTimeOutThreold || vx <= kMinVelocityX) [[unlikely]]
            break;
    }

    if (x >= d && x > prev_x) {
        double ratio = (d - prev_x) / (x - prev_x);
        return { std::lerp(prev_y, y, ratio), std::lerp(prev_t, t, ratio) };
    }
    return { y, t };
}
