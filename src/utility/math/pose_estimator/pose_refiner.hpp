#pragma once

#include <cmath>
#include <concepts>
#include <eigen3/Eigen/Geometry>
#include <type_traits>

namespace rmcs::pose_estimator {

struct Pose {
    Eigen::Vector3d translation { Eigen::Vector3d::Zero() };
    Eigen::Quaterniond orientation { Eigen::Quaterniond::Identity() };
};

template <typename F>
concept PoseCostFunction = std::invocable<F, Pose const&>
    && std::convertible_to<std::invoke_result_t<F, Pose const&>, double>;

template <typename F>
concept UniCostFunction = requires(F f, double v) {
    { f(v) } -> std::convertible_to<double>;
};

class PoseRefiner {
    static constexpr double kGoldenRatioInv { 0.618033988749895 };
    static constexpr double kEarlyStopEpsilon { 1e-4 };
    static constexpr int kMaxSearchIterations { 15 };
    static constexpr double kPitchFreezeThreshold { 1e-4 };
    static constexpr double kSecondPassShrinkFactor { 0.3 };
    static constexpr double kPitchTargetBlendWeight { 0.1 }; // 0=信任 PnP，1=信任先验

public:
    template <UniCostFunction F>
    static auto golden_section_search(double low, double high, F&& f,
        double eps = kEarlyStopEpsilon, int max_iter = kMaxSearchIterations) -> double {
        const double inv_phi = kGoldenRatioInv;

        auto x1 = high - inv_phi * (high - low);
        auto x2 = low + inv_phi * (high - low);
        auto f1 = f(x1);
        auto f2 = f(x2);

        for (int i = 0; i < max_iter; ++i) {
            if (std::abs(high - low) < eps) [[unlikely]] {
                break;
            }

            if (f1 < f2) {
                high = x2;
                x2   = x1;
                f2   = f1;
                x1   = high - inv_phi * (high - low);
                f1   = f(x1);
            } else {
                low = x1;
                x1  = x2;
                f1  = f2;
                x2  = low + inv_phi * (high - low);
                f2  = f(x2);
            }
        }
        return (low + high) * 0.5;
    }

    /**
     * @brief 坐标下降法姿态
     * @param pose 需要优化的姿态（translation 为常量参考，orientation 会被更新）
     * @param yaw_range Yaw 搜索半径 (rad)
     * @param target_pitch Pitch 目标先验 (rad)
     * @param pitch_range Pitch 搜索半径 (rad, 为 0 则固定 Pitch)
     * @param cost_function 代价函数模板
     */
    template <PoseCostFunction CostFn>
    static auto refine(Pose& pose, double yaw_range, double target_pitch, double pitch_range,
        CostFn&& cost_function) -> void {
        const auto R0       = pose.orientation.toRotationMatrix();
        auto current_yaw    = std::atan2(-R0(2, 0), R0(0, 0));
        auto current_pitch  = std::atan2(-R0(1, 2), R0(1, 1));
        auto pitch_center   = current_pitch;
        if (pitch_range < kPitchFreezeThreshold) {
            pitch_center  = target_pitch;
            current_pitch = target_pitch;
        } else {
            pitch_center = (1.0 - kPitchTargetBlendWeight) * current_pitch
                + kPitchTargetBlendWeight * target_pitch;
            current_pitch = pitch_center;
        }

        auto evaluate_cost = [&](double yaw, double pitch) -> double {
            auto candidate = Pose { .translation = pose.translation,
                .orientation = (Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ())
                    * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitX())) };
            return cost_function(candidate);
        };

        {
            current_yaw =
                golden_section_search(current_yaw - yaw_range, current_yaw + yaw_range,
                    [&](double yaw) -> double { return evaluate_cost(yaw, current_pitch); });

            if (pitch_range > kPitchFreezeThreshold) {
                current_pitch =
                    golden_section_search(pitch_center - pitch_range, pitch_center + pitch_range,
                        [&](double pitch) -> double { return evaluate_cost(current_yaw, pitch); });
            }
        }
        {
            current_yaw = golden_section_search(
                current_yaw - yaw_range * kSecondPassShrinkFactor,
                current_yaw + yaw_range * kSecondPassShrinkFactor,
                [&](double yaw) -> double { return evaluate_cost(yaw, current_pitch); });

            if (pitch_range > kPitchFreezeThreshold) {
                current_pitch = golden_section_search(
                    pitch_center - pitch_range * kSecondPassShrinkFactor,
                    pitch_center + pitch_range * kSecondPassShrinkFactor,
                    [&](double pitch) -> double { return evaluate_cost(current_yaw, pitch); });
            }
        }

        pose.orientation = (Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ())
            * Eigen::AngleAxisd(current_yaw, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(current_pitch, Eigen::Vector3d::UnitX()));
    }
};
}
