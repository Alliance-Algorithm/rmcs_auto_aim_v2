#pragma once

#include <eigen3/Eigen/Geometry>
#include <optional>
#include <span>
#include <yaml-cpp/yaml.h>

#include "utility/math/angle.hpp"
#include "utility/pimpl.hpp"
#include "utility/robot/armor.hpp"

namespace rmcs::fire_control {

class AimPointChooser {
    RMCS_PIMPL_DEFINITION(AimPointChooser)

public:
    struct Config {
        double coming_angle { util::deg2rad(70.0) };          // rad
        double leaving_angle { util::deg2rad(20.0) };         // rad
        double outpost_coming_angle { util::deg2rad(70.0) };  // rad
        double outpost_leaving_angle { util::deg2rad(30.0) }; // rad
    };

    auto initialize(Config const& config) noexcept -> void;

    auto choose_armor(std::span<Armor3D const> armors, Eigen::Vector3d const& center_position,
        double angular_velocity) -> std::optional<Armor3D>;
};

} // namespace rmcs::fire_control
