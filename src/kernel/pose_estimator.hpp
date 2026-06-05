#pragma once

#include "utility/image/image.hpp"
#include "utility/pimpl.hpp"
#include "utility/robot/armor.hpp"

#include <array>
#include <expected>
#include <yaml-cpp/yaml.h>

namespace rmcs::kernel {

class PoseEstimator {
    RMCS_PIMPL_DEFINITION(PoseEstimator)

public:
    struct Addition {
        Armor3ds origin;
        std::vector<cv::Rect2i> areas;
        Point2d center;
        Lightbar2ds predicted_near;
        Lightbar2ds predicted_away;
        Lightbar2ds detected_2d;
        Lightbar3ds detected_3d;
    };

    auto initialize(const YAML::Node&) noexcept -> std::expected<void, std::string>;

    auto configure_camera(std::array<double, 9>, std::array<double, 5>) -> void;
    auto update_camera_transform(const Transform& transform) -> void;

    auto estimate_armor(const std::vector<Armor2d>&) const -> Armor3ds;
    auto estimate_armor(const std::vector<Armor2d>&, Image&) const -> Armor3ds;

    auto addition() -> const Addition&;

    auto into_odom_link(std::span<const Armor3d> armors) const -> Armor3ds;
    auto into_odom_link(const Armor3d& armor) const -> Armor3d;
};

}
