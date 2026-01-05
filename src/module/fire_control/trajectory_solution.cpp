#include "trajectory_solution.hpp"

#include <cmath>

using namespace rmcs::fire_control;
/**
 * @brief 弹道解算 (考虑空气阻力迭代)
 * @param v0 子弹初速度 (m/s)
 * @param target_d 水平距离 (m)
 * @param target_h 目标竖直高度 (m) - 目标在枪口上方为正
 * @param g 重力加速度
 * @param params 系数
 * 参考资料：https://zhuanlan.zhihu.com/p/1970271417149920247
 */
auto TrajectorySolution::solve() const -> std::optional<Output> {
    if (input.v0 <= 0 || input.target_d <= 0) return std::nullopt;

    // 实际参与计算的阻力系数 = 基础系数 * 动态补偿
    const double k_effective = params.k * params.bias_scale;

    // 1. 初始猜测值：使用解析解或简单的几何角度
    double pitch = std::atan2(input.target_h, input.target_d);

    // 2. 迭代反馈：修正由于空气阻力导致的下坠偏差
    for (int i = 0; i < kMaxIterateCount; ++i) {
        auto [actual_h, t] = Estimate(input.v0, pitch, input.target_d, k_effective, params.g);

        auto h_error = input.target_h - actual_h;
        if (std::abs(h_error) < kHeightErrorThreold) {
            auto result     = Output {};
            result.fly_time = t;
            result.pitch    = pitch;
            return result;
        }

        // 修正角度：利用线性近似提高收敛速度
        // Δpitch ≈ Δh / d
        pitch += std::atan2(h_error, input.target_d);

        if (std::abs(pitch) > kMaxPitchThreold) break; // 仰角过大保护
    }

    return std::nullopt;
}

/**
 * @brief 弹道前向仿真（数值积分）
 * 计算在给定仰角下，飞行到水平距离 d 时的高度和时间
 */
auto TrajectorySolution::Estimate(double v0, double pitch, double d, double k, double g) const
    -> std::tuple<double, double> {
    double x = 0, y = 0, t = 0;
    double vx = v0 * std::cos(pitch);
    double vy = v0 * std::sin(pitch);

    while (x < d) {
        // F = -k * v * v_vec => a = -k * v * v_vec
        double v = std::sqrt(vx * vx + vy * vy);

        // 速度更新：dv = a * dt
        vx -= k * v * vx * kEstimateDeltaTime;
        vy -= (g + k * v * vy) * kEstimateDeltaTime;

        // 位移更新
        x += vx * kEstimateDeltaTime;
        y += vy * kEstimateDeltaTime;
        t += kEstimateDeltaTime;

        if (t > kEstimateTimeOutThreold) break; // 超时保护
    }

    // 线性插值修正
    // 数值积分最后一步通常会超过 d，通过插值回到精确的 d 位置提高 t 和 y 的精度
    double last_x  = x - vx * kEstimateDeltaTime;
    double ratio   = (d - last_x) / (x - last_x);
    double final_y = (y - vy * kEstimateDeltaTime) + (y - (y - vy * kEstimateDeltaTime)) * ratio;
    double final_t = (t - kEstimateDeltaTime) + kEstimateDeltaTime * ratio;

    return { final_y, final_t };
}
