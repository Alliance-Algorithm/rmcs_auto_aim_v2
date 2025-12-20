#pragma once

#include <chrono>

#include "utility/pimpl.hpp"
#include "utility/robot/armor.hpp"

namespace rmcs::predictor {
struct RobotState {
    RMCS_PIMPL_DEFINITION(RobotState)

    auto initialize(Armor3D const&, std::chrono::steady_clock::time_point const&) -> auto;

    auto predict(std::chrono::steady_clock::time_point const& t) -> void;

    auto update(Armor3D const& armor) -> void;

    auto is_convergened() const -> bool;
};

}
