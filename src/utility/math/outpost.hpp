#pragma once
#include "utility/robot/armor.hpp"

/// 前哨站按照下面的顺序排列，其旋向确定
/// [ UPPER ]
///             [ MIDDLE ]
///                          [ LOWER ]
namespace rmcs::util {

auto outpost_relative_height(bool in_right, bool in_upper) noexcept -> double;

class OutpostSolution {
public:
    enum class ArmorLevel { UPPER, MIDDLE, LOWER };

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
