#pragma once

#include <eigen3/Eigen/Dense>
#include <expected>
#include <limits>
#include <string>
#include <yaml-cpp/yaml.h>

#include "utility/pimpl.hpp"

namespace rmcs::fire_control {

class ShootEvaluator {
    RMCS_PIMPL_DEFINITION(ShootEvaluator)

public:
    struct Command {
        bool control { false };
        bool auto_aim_enabled { false };
        double yaw {
            std::numeric_limits<double>::quiet_NaN(),
        };
        Eigen::Vector3d center_position { Eigen::Vector3d::Zero() };
        Eigen::Vector3d aim_point_position { Eigen::Vector3d::Zero() };
    };

    auto configure_yaml(const YAML::Node& yaml) noexcept -> std::expected<void, std::string>;

    auto evaluate(Command const& command, double current_yaw) noexcept -> bool;
};

} // namespace rmcs::fire_control
