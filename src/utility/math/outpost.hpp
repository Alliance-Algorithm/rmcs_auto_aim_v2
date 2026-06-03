#pragma once
#include "utility/robot/armor.hpp"
#include <utility>

namespace rmcs::util {

class NeighborBarSolution {
public:
    struct Input {
        Armor3D source;
        bool in_right = false;
        double armor_thickness = 0;
    } input;

    struct Result {
        using Bar = std::pair<Point3d, Point3d>;

        Bar upper_near;
        Bar upper_away;

        Bar lower_near;
        Bar lower_away;

        Point3d center;
    } result;

    auto solve() -> void;
};

}
