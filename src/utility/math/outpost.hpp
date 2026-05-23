#pragma once
#include "utility/robot/armor.hpp"

#include <array>
#include <utility>

namespace rmcs::util {

class NeighborBarSolution {
public:
    struct Input {
        Armor3D source;
        bool in_right = false;
    } input;

    struct Result {
        using Bar = std::pair<Point3d, Point3d>;
        std::array<Bar, 2> bars;
    } result;

    auto solve() -> void;
};

}
