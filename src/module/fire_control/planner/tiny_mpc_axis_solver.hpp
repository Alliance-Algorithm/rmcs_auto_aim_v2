#pragma once

#include <expected>
#include <string>

#include "module/fire_control/planner/mpc_types.hpp"

#include "utility/pimpl.hpp"

namespace rmcs::fire_control {

class TinyMpcAxisSolver {
public:
    struct AngularKinematics {
        double angle { 0.0 };
        double rate { 0.0 };
        double acc { 0.0 };
    };

    struct Config {
        double max_acc { 0.0 };
        double q_angle { 0.0 };
        double q_rate { 0.0 };
        double r_acc { 0.0 };
        int max_iter { 10 };
    };

    auto initialize(Config const& config) -> std::expected<void, std::string>;
    auto solve_center(MpcAxisTrajectory const& reference) -> std::expected<double, std::string>;
    auto solve_center_kinematics(MpcAxisTrajectory const& reference)
        -> std::expected<AngularKinematics, std::string>;

    RMCS_PIMPL_DEFINITION(TinyMpcAxisSolver)
};

} // namespace rmcs::fire_control
