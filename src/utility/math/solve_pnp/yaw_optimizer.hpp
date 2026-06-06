#pragma once

#include <array>

#include "utility/math/camera.hpp"
#include "utility/math/linear.hpp"
#include "utility/robot/armor.hpp"
#include "utility/robot/id.hpp"

namespace rmcs::util {

struct YawOptimizer {
    /// @TODO:
    ///  换成标准 Armor 类型
    struct Input {
        CameraFeature camera;
        std::array<Point3d, 4> armor_shape;
        std::array<Point2d, 4> detected_corners;
        Translation xyz_in_world;
        double center_yaw { 0.0 };
        DeviceId genre { DeviceId::UNKNOWN };
    } input;

    struct Output {
        Orientation orientation;
    };

    YawOptimizer() noexcept = default;

    auto solve() -> Output;
};

struct ReprojectionOptimizer {
    struct Input {
        CameraFeature camera;
        Armor2d armor2d;
        Armor3d armor3d;
    } input;

    struct Result {
        Armor3d armor3d;
    } result;

    auto solve() -> bool;
};

} // namespace rmcs::util
