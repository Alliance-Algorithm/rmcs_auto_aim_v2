#pragma once
#include "utility/robot/armor.hpp"

namespace rmcs::util {

class NeighborBarSolution {
public:
    struct Input {
        Armor3d source;
        bool in_right          = false;
        double armor_thickness = 0;
    } input;

    struct Result {
        Lightbar3d upper_near;
        Lightbar3d upper_away;

        Lightbar3d lower_near;
        Lightbar3d lower_away;

        Point3d center;
    } result;

    auto solve() -> void;
};

}
