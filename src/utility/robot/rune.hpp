#pragma once
#include "utility/math/linear.hpp"
#include <array>

namespace rmcs {

struct RuneBullseye {
    Point2d center;
    std::array<Point2d, 4> corners;
    bool active;
    double score;
};
struct RuneIcon {
    Point2d center;
    double score;
};

}
