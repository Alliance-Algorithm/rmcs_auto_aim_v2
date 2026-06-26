#pragma once
#include "utility/math/linear.hpp"
#include "utility/robot/armor.hpp"

namespace rmcs {

struct RobotSolution {
    struct Input {
        // 机器人的中心
        Translation center;

        // 机器人正向的朝向，注意不是装甲板的朝向，装甲板的朝向定义是
        // 从装甲板指向旋转中心，即背后的朝向，这里是旋转中心指向前方
        // 装甲板的方向
        Orientation toward;

        // 前向的装甲板装配半径
        double radius_forward;

        // 侧向的装甲板装配半径
        double radius_lateral;

        // 侧向装甲板相对正向装甲板的高度偏移
        double height_lateral = 0.0;

        ArmorGenre genre = ArmorGenre::UNKNOWN;
        ArmorColor color = ArmorColor::DARK;
    } input;

    /// @NOTE:
    ///  以 ROS 系的 X 轴为前向，id 顺着 Z 轴向下看的逆时针增大
    ///  对于装甲板，0 为 +x 轴，1 为 +y 轴，而对于灯条，0 1 为
    ///  +x 轴装甲板的两个灯条，逆时针递增，即 0 在 +X -Y 象限

    auto solve_armors() -> std::array<Armor3d, 4>;

    auto solve_lightbars() -> std::array<Lightbar3d, 8>;
};

}
