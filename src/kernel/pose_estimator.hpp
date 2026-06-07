#pragma once
#include "utility/image/image.hpp"
#include "utility/pimpl.hpp"
#include "utility/robot/armor.hpp"

#include <expected>
#include <yaml-cpp/yaml.h>

namespace rmcs::kernel {

class PoseEstimator {
    RMCS_PIMPL_DEFINITION(PoseEstimator)

public:
    auto initialize(const YAML::Node&) noexcept -> std::expected<void, std::string>;
    auto update_camera_transform(const Transform& transform) -> void;

    auto estimate_armor(const std::vector<Armor2d>&) const -> std::vector<Armor3d>;
    auto estimate_armor(const std::vector<Armor2d>&, Image&) const -> std::vector<Armor3d>;

    auto into_odom_link(std::span<const Armor3d> armors) const -> std::vector<Armor3d>;
    auto into_odom_link(const Armor3d& armor) const -> Armor3d;

    auto draw_debug(Image&) -> void;
    auto publish_debug() -> void;
};

}
