#include "trajectory_solution.hpp"

#include <cmath>
#include <tuple>

using namespace rmcs;

namespace details {

constexpr auto kMaxIterateCount          = int { 10 };
constexpr auto kMaxPitchThreold          = double { 80.0 / 57.3 };
constexpr auto kEstimateDeltaTime        = double { 0.005 };
constexpr auto kHeightErrorThreold       = double { 0.001 };
constexpr auto kEstimateTimeOutThreold   = double { 4.0 };
constexpr auto kMinVelocityX             = double { 0.1 };
constexpr auto kGravity                  = double { 9.81 };
constexpr auto kAirResistanceCoefficient = double { 0.003 };

constexpr auto estimate(double v0, double pitch, double d, double air_resistance)
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

}

auto TrajectorySolution::solve() const -> std::optional<Output> {

    const auto target_d = std::hypot(input.point.x, input.point.y);
    const auto target_h = input.point.z;

    if (input.v0 <= 0 || target_d <= 0) return std::nullopt;

    const auto yaw = std::atan2(input.point.y, input.point.x);

    double pitch = std::atan2(target_h, target_d);
    for (int i = 0; i < details::kMaxIterateCount; ++i) {
        auto [actual_h, t] =
            details::estimate(input.v0, pitch, target_d, details::kAirResistanceCoefficient);

        auto h_error = target_h - actual_h;
        if (std::abs(h_error) < details::kHeightErrorThreold) {
            auto result     = Output { };
            result.fly_time = t;
            result.yaw      = yaw;
            result.pitch    = -pitch;
            return result;
        }

        pitch += std::atan2(h_error, target_d);

        if (std::abs(pitch) > details::kMaxPitchThreold) break;
    }

    return std::nullopt;
}
