#pragma once

#include <eigen3/Eigen/Dense>
#include <expected>
#include <yaml-cpp/yaml.h>

#include "module/fire_control/planner/tiny_mpc_axis_solver.hpp"
#include "utility/pimpl.hpp"

namespace rmcs::fire_control {

// row 0: yaw angle (rad)
// row 1: yaw angular velocity (rad/s)
// row 2: pitch angle (rad)
// row 3: pitch angular velocity (rad/s)
using ReferenceTrajectory = Eigen::Matrix<double, 4, kMpcAxisHorizon>;

class MpcTrajectoryPlanner {
    RMCS_PIMPL_DEFINITION(MpcTrajectoryPlanner)

public:
    struct Plan {
        double yaw { 0.0 };
        double pitch { 0.0 };
    };

    auto configure_yaml(const YAML::Node& yaml) noexcept -> std::expected<void, std::string>;

    auto plan(ReferenceTrajectory const& reference) -> std::expected<Plan, std::string>;
};

} // namespace rmcs::fire_control
