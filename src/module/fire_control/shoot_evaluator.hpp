#pragma once

#include <expected>
#include <string>

#include "utility/pimpl.hpp"

namespace rmcs::fire_control {

class ShootEvaluator {
    RMCS_PIMPL_DEFINITION(ShootEvaluator)

public:
    struct Config {
        double first_tolerance { 4.0 };  // rad
        double second_tolerance { 2.0 }; // rad
        double judge_distance { 3.0 };   // m
        bool auto_fire { true };
    };

    struct Command {
        bool control { false };
        bool auto_aim_enabled { false };
        bool aim_point_valid { false };
        double yaw { 0. };
        double distance { 0. };
    };

    auto initialize(Config const& config) noexcept -> std::expected<void, std::string>;

    auto evaluate(Command const& command, double current_yaw) noexcept -> bool;
};

} // namespace rmcs::fire_control
