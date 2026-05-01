#pragma once

#include <expected>
#include <yaml-cpp/yaml.h>

#include "utility/pimpl.hpp"
#include "utility/robot/armor.hpp"

namespace rmcs::kernel {

class PoseEstimator {
    RMCS_PIMPL_DEFINITION(PoseEstimator)

public:
    auto initialize(const YAML::Node&) noexcept -> std::expected<void, std::string>;
    auto update_camera_transform(Transform const& transform) -> void;

    auto estimate_armor(std::vector<Armor2D> const&) const -> std::vector<Armor3D>;

    auto into_odom_link(std::span<Armor3D const> armors) const -> std::vector<Armor3D>;
    auto into_odom_link(Armor3D const& armor) const -> Armor3D;
};

}
