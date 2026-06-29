#pragma once

#include <eigen3/Eigen/Core>

#include <expected>
#include <limits>
#include <string>

#include <yaml-cpp/yaml.h>

#include "module/fire_control/gimbal_state.hpp"
#include "utility/pimpl.hpp"

namespace rmcs::fire_control {

class ShootEvaluator {
    RMCS_PIMPL_DEFINITION(ShootEvaluator)

public:
    struct Command {
        double yaw { std::numeric_limits<double>::quiet_NaN() };
        double pitch { std::numeric_limits<double>::quiet_NaN() };
        Eigen::Vector3d center { Eigen::Vector3d::Zero() };
        Eigen::Vector3d attack { Eigen::Vector3d::Zero() };
    };

    auto initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string>;
    auto configure_yaml(const YAML::Node& yaml) noexcept -> std::expected<void, std::string>;

    auto evaluate(Command const& command, GimbalState const& state) noexcept -> bool;
};

} // namespace rmcs::fire_control
