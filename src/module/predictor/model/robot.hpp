#pragma once

#include "utility/math/camera.hpp"
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

        util::CameraFeature camera_feature;
    };

    explicit RobotModel(std::span<const Armor2d>, const Config&) noexcept;

    auto configure(const Config&) noexcept -> void;

    auto predict(double dt) noexcept -> void;
    auto correct(std::span<const Armor2d>, std::span<const Lightbar2d>) noexcept -> void;

    auto state() noexcept -> State;

    auto full() const -> std::array<Armor3d, 4>;

    struct Addition {
        struct Tracked {
            int lightbar_id;
            Point2d point;
        };
        std::vector<Tracked> tracked;
    };
    auto addition() const -> const Addition&;
};

}
