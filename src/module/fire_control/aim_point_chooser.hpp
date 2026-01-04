#pragma once

#include <eigen3/Eigen/Geometry>
#include <expected>
#include <yaml-cpp/yaml.h>

#include "utility/pimpl.hpp"
#include "utility/robot/armor.hpp"

namespace rmcs::fire_control {
class AimPointChooser {
    RMCS_PIMPL_DEFINITION(AimPointChooser)

public:
    auto initialize(const YAML::Node&) noexcept -> std::expected<void, std::string>;

    auto choose_armor(std::span<Armor3D const> armors, Eigen::Vector<double, 11> const& ekf_x)
        -> std::optional<Armor3D>;
};

} // namespace rmcs::fire_control
