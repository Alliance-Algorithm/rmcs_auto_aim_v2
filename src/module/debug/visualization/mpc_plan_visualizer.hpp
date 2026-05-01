#pragma once

#include "utility/pimpl.hpp"
#include "utility/rclcpp/node.hpp"

namespace rmcs::debug {

class MpcPlanVisualizer {
    RMCS_PIMPL_DEFINITION(MpcPlanVisualizer)

public:
    auto initialize(util::RclcppNode& node) noexcept -> void;

    auto publish_planned_yaw(double yaw, double yaw_rate, double yaw_acc) const noexcept -> bool;

    auto publish_planned_pitch(double pitch, double pitch_rate, double pitch_acc) const noexcept
        -> bool;
};

} // namespace rmcs::debug
