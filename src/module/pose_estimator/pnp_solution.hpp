#pragma once

#include <array>

#include "utility/math/linear.hpp"
#include "utility/math/point.hpp"
#include "utility/rclcpp/node.hpp"
#include "utility/rclcpp/visual/armor.hpp"
#include "utility/robot/color.hpp"
#include "utility/robot/id.hpp"

namespace rmcs::util {

struct PnpSolution {
    struct Input {
        // Row Major
        std::array<std::array<double, 3>, 3> camera_matrix;
        std::array<double, 5> distort_coeff;
        std::array<Point3d, 4> armor_shape;
        std::array<Point2d, 4> armor_detection;
        DeviceId genre;
        CampColor color;
    } input;
    struct Result {
        Translation translation;
        Orientation orientation;
        DeviceId genre;
        CampColor color;
    } result;

    std::unique_ptr<visual::Armor> visualized_armor;
    std::unique_ptr<visual::Armor> target_armor;

    PnpSolution() noexcept = default;

    auto solve() noexcept -> void;

    auto visualize(RclcppNode&) noexcept -> void;
};
}
