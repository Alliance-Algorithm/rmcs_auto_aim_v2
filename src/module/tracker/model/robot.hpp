#pragma once

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

        auto transition(double seconds) -> void;

        auto aimpoints() const -> std::vector<Point3d>;
        auto direction() const -> Point3d;
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

    auto update_transform(const Transform&) noexcept -> void;

    auto start_with(std::span<const Armor2d>) noexcept -> bool;
    auto predict(double dt) noexcept -> void;
    auto correct(std::span<const Armor2d>, std::span<const Lightbar2d>) noexcept -> void;

    auto state() const noexcept -> State;

    auto full() const -> std::array<Armor3d, 4>;

    auto converge() const -> bool {
        // TODO: 待实现，并不是我自信到认为该模型一定收敛
        return true;
    }

    auto addition() const -> const Addition&;
};

}
