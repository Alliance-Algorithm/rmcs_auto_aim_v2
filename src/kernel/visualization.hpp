#pragma once

#include <expected>

#include <yaml-cpp/yaml.h>

#include "utility/image/image.hpp"
#include "utility/rclcpp/node.hpp"
#include "utility/robot/armor.hpp"

namespace rmcs::kernel {

class Visualization {
    RMCS_PIMPL_DEFINITION(Visualization)

public:
    static constexpr auto get_prefix() noexcept { return "visualization"; }

    auto operator<<(const Image& image) noexcept -> Visualization& {
        return update_image(image), *this;
    }

    auto initialize(const YAML::Node& yaml, util::RclcppNode& visual_node) noexcept
        -> std::expected<void, std::string>;

    auto initialized() const noexcept -> bool;

    auto update_image(const Image& image) noexcept -> bool;

    auto update_visible_armors(std::span<Armor3D const> armors) const -> bool;

    auto update_visible_robot(std::span<Armor3D const> armors) const -> bool;

    auto update_aiming_direction(double yaw, double pitch) const -> void;

    // TODO: 整理接口
    auto update_mpc_plan(double yaw, double pitch, double yaw_rate, double pitch_rate,
        double yaw_acc, double pitch_acc) const -> void;

    auto update_camera_pose(const Orientation&) const -> void;
};

} // namespace rmcs::kernel
