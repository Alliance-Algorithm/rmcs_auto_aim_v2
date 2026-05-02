#pragma once

#include <eigen3/Eigen/Geometry>

#include <expected>
#include <optional>
#include <span>
#include <string>

#include <yaml-cpp/yaml.h>

#include "utility/pimpl.hpp"
#include "utility/robot/armor.hpp"

namespace rmcs::fire_control {

class AimPointChooser {
    RMCS_PIMPL_DEFINITION(AimPointChooser)

public:
    struct Config {
        double coming_angle;
        double leaving_angle;
        double angular_velocity_threshold;
        double outpost_coming_angle;
        double outpost_leaving_angle;
    };

    auto initialize(Config const& config) noexcept -> void;
    auto configure_yaml(const YAML::Node& yaml) noexcept -> std::expected<void, std::string>;

    auto choose_armor(std::span<Armor3D const> armors, Eigen::Vector3d const& center_position,
        double angular_velocity) -> std::optional<Armor3D>;
};

} // namespace rmcs::fire_control
