#pragma once

// TODO: 旋转轴从相机系变换到 odom 系（当前假设 camera≡odom，旋转轴垂水平面）
//       方案 A: correct() 入口用 camera_transform 把观测变到 odom 系
//       方案 B: State 增加旋转轴方向（3维），扩展 Jacobian ∂p3d/∂axis
//
// TODO: 接入邻侧灯条识别（参考前哨站 pose_estimator 的邻侧灯条算法）
//       提取 Lightbar2d 列表，喂给 correct(armors_2d, lightbars_2d)

#include "utility/pimpl.hpp"
#include "utility/robot/armor.hpp"

namespace rmcs {

class RobotModel {
    RMCS_PIMPL_DEFINITION(RobotModel)

public:
    struct State {
        // 机器人中心坐标
        double x;
        double y;
        double z;

        // 机器人平移速度
        double vx = 0;
        double vy = 0;
        double vz = 0;

        // 机器人旋转速度及角度值
        double rotation_angle;
        double rotation_speed;

        // 机器人物理尺寸
        double radius_forward = 0.2;
        double radius_lateral = 0.2;

        double height_lateral = 0.0;
    };
    struct Config {
        static constexpr double kRadiusMin = 0.1;
        static constexpr double kRadiusMax = 0.4;

        double radius_forward_min = kRadiusMin;
        double radius_forward_max = kRadiusMax;

        double radius_lateral_min = kRadiusMin;
        double radius_lateral_max = kRadiusMax;

        double height_lateral_min = -0.2;
        double height_lateral_max = +0.2;

        double rotation_speed_min = std::numbers::pi * 0;
        double rotation_speed_max = std::numbers::pi * 8;

        double yaw_full_max = 60.0 * std::numbers::pi / 180.0;
        double yaw_part_max = 90.0 * std::numbers::pi / 180.0;

        // 过程噪声
        double noise_x = 1e-4;
        double noise_y = 1e-4;
        double noise_z = 1e-4;

        double noise_vx = 5e-3;
        double noise_vy = 5e-3;
        double noise_vz = 5e-3;

        double noise_rotation_angle = 5e-4;
        double noise_rotation_speed = 1e-2;

        double noise_radius_forward = 1e-8;
        double noise_radius_lateral = 1e-8;

        double noise_height_lateral = 1e-8;

        // 观测噪声（像素噪声）
        double noise_observation = 40.0;
    };
    struct Addition {
        struct Tracked {
            int lightbar_id;
            Point2d point;
        };
        std::vector<Tracked> tracked;

        std::array<Armor2d, 4> armors;
    };

    explicit RobotModel(const Config&) noexcept;

    auto configure(const Config&) noexcept -> void;

    auto configure_camera(std::array<double, 9>, std::array<double, 5>) noexcept -> void;

    auto start_with(std::span<const Armor2d>) noexcept -> void;

    auto predict(double dt) noexcept -> void;
    auto correct(std::span<const Armor2d>, std::span<const Lightbar2d>) noexcept -> void;

    auto state() noexcept -> State;

    auto full() const -> std::array<Armor3d, 4>;

    auto addition() const -> const Addition&;
};

}
