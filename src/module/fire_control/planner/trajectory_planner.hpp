#pragma once

#include <expected>
#include <string>

#include <yaml-cpp/yaml.h>

#include "module/fire_control/planner/mpc_types.hpp"
#include "module/fire_control/types.hpp"
#include "module/predictor/snapshot.hpp"
#include "utility/pimpl.hpp"

namespace rmcs::fire_control {

class TrajectoryPlanner {
    RMCS_PIMPL_DEFINITION(TrajectoryPlanner)

public:
    struct Plan {
        double yaw { 0.0 };
        double pitch { 0.0 };
        double yaw_rate { 0.0 };
        double pitch_rate { 0.0 };
        double yaw_acc { 0.0 };
        double pitch_acc { 0.0 };
    };

    auto configure_yaml(const YAML::Node& yaml) noexcept -> std::expected<void, std::string>;

    auto plan(predictor::Snapshot const& snapshot, TargetSolution const& raw_solution,
        TimePoint command_time, double bullet_speed, double shoot_delay)
        -> std::expected<Plan, std::string>;
};

} // namespace rmcs::fire_control
