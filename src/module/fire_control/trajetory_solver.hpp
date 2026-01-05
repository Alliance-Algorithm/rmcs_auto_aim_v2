#pragma once

#include <optional>
#include <tuple>

namespace rmcs::fire_control {
struct TrajectorySolution {

    struct TrajectoryParams {
        double k { 0.019 };       // 基础阻力系数 (小弹丸~0.019, 大弹丸~0.005)
        double g { 9.81 };        // 重力加速度 (通常取 9.78~9.8)
        double bias_scale { 1. }; // 动态补偿系数：修正额外阻力,阻力越大，该系数越大，default=1
    } params;

    struct Input {
        double v0 { 0. };
        double target_d { 0. };
        double target_h { 0. };
        TrajectoryParams params;
    } input;

    struct Output {
        double fly_time { 0. }; // s
        double pitch { 0. };    // rad
    } result;

    auto solve() const -> std::optional<Output>;

private:
    auto Estimate(double v0, double pitch, double d, double k, double g) const
        -> std::tuple<double, double>;

    const int kMaxIterateCount { 10 };
    const double kMaxPitchThreold { 57.3 / 57.3 }; // rad

    const double kEstimateDeltaTime { 0.005 };
    const double kHeightErrorThreold { 0.001 };
    const double kEstimateTimeOutThreold { 4.0 };
};
}
