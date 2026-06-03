#pragma once

#include "utility/math/camera.hpp"
#include "utility/robot/armor.hpp"

namespace rmcs::util {

struct OutpostDistanceOptimizer {
    struct Input {
        Armor3D initial;

        Armor2D armor;
        Point2d upper_point;
        Point2d lower_point;

        bool is_right { false };
        bool is_upper { false };

        double armor_thickness = 0;

        CameraFeature camera;
    } input;

    struct Result {
        Armor3D armor;
    } result;

    OutpostDistanceOptimizer() noexcept = default;

    auto solve() -> bool;
};

}
