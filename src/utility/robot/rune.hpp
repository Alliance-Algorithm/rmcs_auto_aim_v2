#pragma once
#include "utility/math/linear.hpp"

#include <array>

namespace rmcs {

struct RunePage {
    bool active = false;

    Point2d center;
    std::array<Point2d, 4> gap_corners;
    std::array<bool, 4> gap_valid{};
};

struct RuneTarget {
    Transform transform;

    /// ......
};

}
