#pragma once
#include "utility/linear.hpp"
#include <array>
#include <tuple>

namespace rmcs::util {

struct ArmorsForwardSolution {
    struct Input {
        double robot_width  = 0.5;
        double robot_height = 0.5;
        Translation t;
        Orientation q;
    } input;

    struct Result {
        using ArmorStatus = std::tuple<Translation, Orientation>;
        std::array<ArmorStatus, 4> armors_status;
    } result;

    ArmorsForwardSolution() = default;

    auto solve() noexcept -> void;
};
struct ArmorsReverseSolution { };

}
