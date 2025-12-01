#pragma once
#include "utility/math/linear.hpp"
#include "utility/math/point.hpp"
#include <array>

namespace rmcs::util {

struct PnpSolution {
    struct Input {
        // Row Major
        std::array<std::array<double, 3>, 3> camera_matrix;
        std::array<double, 5> distort_coeff;
        std::array<Point3d, 4> armor_shape;
        std::array<Point2d, 4> armor_detection;
    } input;
    struct Result {
        Translation translation;
        Orientation orientation;
    } result;

    PnpSolution() noexcept = default;

    auto solve() noexcept -> void;
};

}
