#pragma once

#include <expected>
#include <optional>
#include <yaml-cpp/yaml.h>

#include "module/fire_control/aim_point_chooser.hpp"
#include "module/predictor/snapshot.hpp"
#include "utility/pimpl.hpp"

namespace rmcs::fire_control {

class MpcTrajectoryPlanner {
    RMCS_PIMPL_DEFINITION(MpcTrajectoryPlanner)

public:
    struct Plan {
        double target_yaw { 0.0 };
        double target_pitch { 0.0 };
        double yaw { 0.0 };
        double pitch { 0.0 };
    };

    auto initialize(const YAML::Node& yaml, AimPointChooser::Config const& chooser_config) noexcept
        -> std::expected<void, std::string>;

    auto plan(const predictor::Snapshot& snapshot,
        TimePoint center_time, double bullet_speed, double yaw_offset,
        double pitch_offset) -> std::optional<Plan>;
};

} // namespace rmcs::fire_control
