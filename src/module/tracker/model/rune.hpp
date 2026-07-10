#pragma once

#include "utility/math/linear.hpp"
#include "utility/pimpl.hpp"
#include "utility/robot/aimpoint.hpp"
#include "utility/robot/rune.hpp"

#include <array>
#include <numbers>
#include <span>

namespace rmcs {

class RuneModel {
    RMCS_PIMPL_DEFINITION(RuneModel)

public:
    // 小符点亮 1 块，大符点亮 2 块，由初始化时点亮数量分类
    enum class Kind : std::uint8_t { Small, Big };

    struct State {
        // EKF 状态量
        double cx             = 0.0;
        double cy             = 0.0;
        double cz             = 0.0; // 符盘中心（Odom）
        double plane_yaw      = 0.0; // 符盘法向绕世界 Z 轴朝向
        double rotation_angle = 0.0; // 参考相位 θ（YZ 平面内自转）
        double rotation_speed = 0.0; // 瞬时 ω

        // 逻辑状态量
        Kind kind               = Kind::Small; // Small / Big
        double t_active         = 0.0; // 大符正弦计时
        double sin_a            = 0.0; // 大符正弦幅值 a（独立拟合）
        double sin_omega        = 0.0; // 大符正弦角频率 ω_sin（独立拟合）
        std::array<bool, 5> lit = { }; // 5 扇叶点亮，随 index 绑定

        auto transition(double seconds) -> void;

        auto get_direction() const -> Point3d;
        auto get_aimpoints() const -> AimPoints;
        auto get_rotation_speed() const -> double;
    };

    struct Config {
        // 过程噪声（符盘中心/朝向近乎静止，取极小值以强平滑）
        double process_noise_xy        = 1e-4;
        double process_noise_z         = 1e-4;
        double process_noise_plane_yaw = 1e-4;
        double process_noise_angle     = 1e-4;
        double process_noise_speed     = 5e-2;

        // 观测噪声
        double observation_noise_xy        = 0.01;
        double observation_noise_z         = 0.02;
        double observation_noise_plane_yaw = 0.02;
        double observation_noise_angle     = 0.01;

        // 小符标称转速
        double small_speed = std::numbers::pi / 3.0;
    };

    explicit RuneModel(const Config&) noexcept;

    auto update_camera(std::array<double, 9>, std::array<double, 5>) noexcept -> void;
    auto update_transform(const Transform&) noexcept -> void;

    // 初始化：靠靶心十字方向与 R 标关联，点亮数量分类 Small / Big
    auto init(std::span<const RuneBullseye>, std::span<const RuneIcon>) noexcept -> bool;

    auto predict(double dt) noexcept -> void;
    auto correct(std::span<const RuneBullseye>, std::span<const RuneIcon>) noexcept -> void;

    auto converge() const -> bool;
    auto diverged() const -> bool;

    auto state() const noexcept -> State;
};

}
