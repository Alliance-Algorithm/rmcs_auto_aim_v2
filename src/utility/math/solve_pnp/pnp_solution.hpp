#pragma once

#include <array>
#include <vector>

#include "utility/math/camera.hpp"
#include "utility/math/linear.hpp"
#include "utility/robot/armor.hpp"
#include "utility/robot/color.hpp"
#include "utility/robot/id.hpp"

namespace rmcs::util {

struct PnpSolution {
    struct Input {
        CameraFeature camera;
        std::array<Point3d, 4> armor_shape;
        std::array<Point2d, 4> armor_detection;
        DeviceId genre;
        CampColor color;
    } input;

    /// @NOTE: Armor 局部坐标系的 X 轴朝向背面
    ///                              .   ^   .
    ///                              |  (y)  |
    ///  camera  ---------->  [正面] | armor |-(x)-> [背面]
    ///                              |       |
    ///                              .       .
    struct Result {
        Translation translation;
        Orientation orientation;
        DeviceId genre;
        CampColor color;
    } result;

    PnpSolution() noexcept = default;

    auto solve() -> bool;
};

struct RobustPnpSolution {
    struct Input {
        CameraFeature feature;
        Armor2d armor2d;

        bool fixed_outpost_pitch = false;
        bool fixed_normal_pitch  = false;
    } input;

    struct Result {
        /// solve() 内部已经通过 CameraFeature 外参转换到 Odom 坐标系
        /// 因为有一些裁剪分支必须依赖 Odom 系下的信息
        Armor3d armor3d;
    } result;

    auto solve() -> bool;
};

// 单扇叶的 Pnp 变换，只能观测到一个未激活的靶心时，使用该 Solution
struct SingleRunePnpSolution {
    struct Input {
        CameraFeature cam;

        Point2d center;
        Point2d icon;
        std::array<Point2d, 4> corners;
    } input;

    struct Result {
        Translation translation;
        Orientation orientation;
    } result;

    auto solve() -> bool;
};

// 大符或者存在已激活的小符，可以用此 Solution 做大尺度的 Pnp
struct MultipleRunePnpSolution {
    struct Input {
        CameraFeature cam;

        Point2d icon;
        std::vector<Point2d> centers;
    } input;
};

}
