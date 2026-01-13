#pragma once

#include <expected>
#include <yaml-cpp/yaml.h>

#include "utility/math/linear.hpp"
#include "utility/pimpl.hpp"
#include "utility/rclcpp/node.hpp"
#include "utility/robot/armor.hpp"
#include "utility/shared/context.hpp"

namespace rmcs::kernel {

class PoseEstimator {
    RMCS_PIMPL_DEFINITION(PoseEstimator)

public:
    using RclcppNode = util::RclcppNode;

    auto initialize(const YAML::Node&) noexcept -> std::expected<void, std::string>;

    auto visualize(RclcppNode& visual_node) -> void;

    auto solve_pnp(std::vector<Armor2D> const&) const -> std::optional<std::vector<Armor3D>>;

    auto set_odom_to_camera_transform(util::Transform const& transform) -> void;

    auto odom_to_camera(std::span<Armor3D const> armors) const -> std::vector<Armor3D>;
    auto odom_to_camera(Armor3D const& armor) const -> Armor3D;

    auto update_imu_link(const Orientation&) noexcept -> void;
};
}
