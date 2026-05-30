#pragma once

#include <array>

#include "utility/math/camera.hpp"
#include "utility/math/linear.hpp"
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
}
