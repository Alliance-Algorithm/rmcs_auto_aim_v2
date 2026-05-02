#pragma once

#include <eigen3/Eigen/Dense>

#include <expected>
#include <string>

#include "module/fire_control/aim_point_chooser.hpp"
#include "module/predictor/snapshot.hpp"
#include "utility/pimpl.hpp"

namespace rmcs::fire_control {

class TargetSolutionSolver {
    RMCS_PIMPL_DEFINITION(TargetSolutionSolver)

public:
    struct TargetSolution {
        TimePoint impact_time;
        double impact_yaw { 0.0 };
        double impact_pitch { 0.0 };
        Eigen::Vector3d center_position { Eigen::Vector3d::Zero() };
        Eigen::Vector3d aim_point { Eigen::Vector3d::Zero() };
    };

    auto solve(predictor::Snapshot const& snapshot, AimPointChooser& chooser, double bullet_speed,
        double shoot_delay) const -> std::expected<TargetSolution, std::string>;
};

} // namespace rmcs::fire_control
