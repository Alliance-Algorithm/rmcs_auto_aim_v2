#pragma once

#include <array>

#include "utility/math/linear.hpp"
#include "utility/math/point.hpp"
#include "utility/robot/color.hpp"
#include "utility/robot/id.hpp"

namespace rmcs::util {

struct PnpSolution {
    struct Input {
        // Row Major
        std::array<std::array<double, 3>, 3> camera_matrix;
        std::array<double, 5> distort_coeff;
        std::array<Point3d, 4> armor_shape;
        std::array<Point2d, 4> armor_detection;
        DeviceId genre;
        CampColor color;
    } input;
    struct Result {
        Translation translation;
        Orientation orientation;
        DeviceId genre;
        CampColor color;
    } result;

    /**
 * @brief 构造一个空的 PnpSolution 实例。
 *
 * input 和 result 成员保持其各自类型的默认构造状态，便于随后填充输入并调用 solve()。
 */
PnpSolution() noexcept = default;

    auto solve() -> bool;
};
}