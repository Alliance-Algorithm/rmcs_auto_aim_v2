#pragma once
#include "utility/robot/armor.hpp"

namespace rmcs::util {

class OutpostSolution {
public:
    enum class ArmorLevel { UPPER, MIDDLE, LOWER };

    /// Level 按照下面的顺序排列，其旋向确定
    /// [ UPPER ]
    ///             [ MIDDLE ]
    ///                          [ LOWER ]
    struct Input {
        Translation translation;
        Orientation orientation;
        ArmorLevel source;
        ArmorLevel target;

        double armor_thickness = 0;
    } input;

    struct Result {
        Point3d center;
        Translation translation;
        Orientation orientation;
    } result;

    auto solve() -> void;
};

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
