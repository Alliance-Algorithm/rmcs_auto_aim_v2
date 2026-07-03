#pragma once

#include "utility/math/linear.hpp"

#include <optional>

namespace rmcs {

struct TrajectorySolution {
    struct Input {
        double v0 { 0. };
        Point3d point;
    } input;

    struct Output {
        double fly_time { 0. }; // s
        double yaw { 0. }; // rad
        double pitch { 0. }; // rad
    } result;

    auto solve() const -> std::optional<Output>;
};

}
