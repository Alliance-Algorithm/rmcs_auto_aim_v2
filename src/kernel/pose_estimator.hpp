#pragma once

#include "utility/math/linear.hpp"
#include "utility/pimpl.hpp"
#include "utility/rclcpp/node.hpp"
#include "utility/robot/armor.hpp"
#include <expected>
#include <yaml-cpp/yaml.h>

namespace rmcs::kernel {

class PoseEstimator {
    RMCS_PIMPL_DEFINITION(PoseEstimator)

public:
    using RclcppNode = util::RclcppNode;

    auto initialize(const YAML::Node&) noexcept -> std::expected<void, std::string>;

    auto visualize(RclcppNode& visual_node) -> void;

    auto solve_pnp(std::vector<Armor2D> const&) const noexcept
        -> std::optional<std::vector<Armor3D>>;
    auto update_imu_link(const Orientation&) noexcept -> void;
};

}
