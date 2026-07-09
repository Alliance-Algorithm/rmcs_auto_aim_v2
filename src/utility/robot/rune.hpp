#pragma once
#include "utility/math/linear.hpp"
#include <array>

namespace rmcs {

constexpr auto kRuneGlobalRadius   = 0.7; // 能量机关整体的半径
constexpr auto kRuneBullseyeRadius = 0.15; // 取十字端点的末端，最大半径

constexpr auto kRuneIconProminentDistance = 0.1; // R 标突出的距离，参考的是靶心的平面

// 局部坐标系，以大符中心为原点，X 正方向指向背面
struct RunePagePoints {
    static constexpr Point3d kIcon { -kRuneIconProminentDistance, 0., 0. };
    static constexpr Point3d kT { 0, 0, kRuneGlobalRadius + kRuneBullseyeRadius };
    static constexpr Point3d kL { 0, +kRuneBullseyeRadius, kRuneGlobalRadius };
    static constexpr Point3d kB { 0, 0, kRuneGlobalRadius - kRuneBullseyeRadius };
    static constexpr Point3d kR { 0, -kRuneBullseyeRadius, kRuneGlobalRadius };

    static constexpr std::array kPoints { kIcon, kT, kL, kB, kR };
};

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
