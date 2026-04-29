#pragma once

#include <expected>
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
        bool aim_point_valid { false };
        double yaw { 0. };
        double distance { 0. };
    };

    auto configure_yaml(const YAML::Node& yaml) noexcept -> std::expected<void, std::string>;

    auto evaluate(Command const& command, double current_yaw) noexcept -> bool;
};

} // namespace rmcs::fire_control
