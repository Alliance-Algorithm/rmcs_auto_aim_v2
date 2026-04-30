#pragma once

#include <expected>
#include <yaml-cpp/yaml.h>

#include "module/fire_control/planner/mpc_types.hpp"
#include "utility/pimpl.hpp"

namespace rmcs::fire_control {

class MpcTrajectoryPlanner {
    RMCS_PIMPL_DEFINITION(MpcTrajectoryPlanner)

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

    auto plan(ReferenceTrajectory const& reference) -> std::expected<Plan, std::string>;
};

} // namespace rmcs::fire_control
