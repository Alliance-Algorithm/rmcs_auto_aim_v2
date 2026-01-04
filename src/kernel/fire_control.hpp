#pragma once

#include <eigen3/Eigen/Geometry>
#include <expected>
#include <yaml-cpp/yaml.h>

#include "utility/pimpl.hpp"
#include "utility/robot/armor.hpp"

namespace rmcs::kernel {

class FireControl {
    RMCS_PIMPL_DEFINITION(FireControl)

public:
    auto initialize(const YAML::Node&) noexcept -> std::expected<void, std::string>;

    auto choose_armor(std::span<const Armor3D> armors, const Eigen::Vector<double, 11>& ekf_x)
        -> std::optional<Armor3D>;
};
}
