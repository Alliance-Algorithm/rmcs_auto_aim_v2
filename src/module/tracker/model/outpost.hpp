#pragma once

#include "utility/pimpl.hpp"
#include "utility/robot/armor.hpp"

namespace rmcs {

class OutpostModel {
    RMCS_PIMPL_DEFINITION(OutpostModel)

public:
    struct State {
        // 前哨站旋转中心在世界坐标系下的位置
        double x;
        double y;
        double z;

        // 以参考装甲板朝向为基准的角速度与 yaw 角
        double rotation_speed;
        double rotation_angle;

        std::uint8_t index = 0;

        auto transition(double seconds) -> void;

        auto direction() const -> Point3d;
        auto aimpoints() const -> std::vector<Point3d>;
    };

    struct Config {
        /// @brief 过程噪声 (Q) - 决定对物理模型的信任度
        /// @note 前哨站中心在场地上是绝对静止的，这两个值应尽可能小

        // 中心点 XY 轴漂移噪声
        double process_noise_xy = 1e-6;

        // 中心点 Z 轴（高度）漂移噪声
        double process_noise_z = 1e-5;

        /// @note 增大 speed 会加速转速收敛速度（减小跟踪滞后），但过大会导致速度曲线随视觉高频抖动

        // 转速变化（角速度）噪声
        double process_noise_speed = 1e-4;

        // 角度积分累积噪声
        double process_noise_angle = 1e-4;

        /// @brief 观测噪声 (R) - 决定对视觉/PnP 数据的信任度（单位：米 / 弧度）
        /// @note 增大以下各值，会加大对历史轨迹的滤波平滑效果，降低对单帧视觉跳变的敏感度

        // 视觉解算出的中心 XY 轴误差
        double observation_noise_xy = 0.005;

        // 视觉解算出的中心 Z 轴误差
        double observation_noise_z = 0.01;

        // 视觉解算出的装甲板朝向(Yaw)误差
        double observation_noise_yaw = 0.01;

        /// @brief 切板检测判定阈值
        /// @note 三块装甲板夹角 120°，考虑到观测的局限，建议设在 50.0 ~ 80.0 度之间

        // 触发切板判定的最小 Yaw 角跳变（度）
        double plate_switch_yaw_min = 50.0;
    };

    explicit OutpostModel(const Armor3d& armor) noexcept;

    auto configure(const Config&) noexcept -> void;

    auto predict(double dt) noexcept -> void;
    auto correct(const Armor3d& armor) noexcept -> void;

    // 前哨站的状态向量内容喵，参考点为构造时从传入的那块
    auto state() noexcept -> State;

    // 所有前哨站的装甲板，按照从高到低的顺序排列
    // 也就是说，0 是最高的那块
    auto full() const -> std::array<Armor3d, 3>;

    // 当前正在观测的那块装甲板（即上一个 correct 的装甲板）
    // 注意纯预测不会改变 current 返回的装甲板，即使预测后准
    // 备返回的装甲板在前哨站背面
    auto current() const -> Armor3d;

    auto converge() const -> bool {
        // TODO: 待实现，并不是我自信到认为该模型一定收敛
        return std::ignore = this, true;
    }
};

}
