#pragma once

#include <expected>
#include <string>

#include <eigen3/Eigen/Geometry>

#include "utility/pimpl.hpp"

namespace rmcs::fire_control {

inline constexpr int kMpcAxisHorizon = 100;

using MpcAxisTrajectory = Eigen::Matrix<double, 2, kMpcAxisHorizon>;

class TinyMpcAxisSolver {
    RMCS_PIMPL_DEFINITION(TinyMpcAxisSolver)

public:
    struct Config {
        double max_acc { 0.0 };
        double q_angle { 0.0 };
        double q_rate { 0.0 };
        double r_acc { 0.0 };
        int max_iter { 10 };
    };

    auto initialize(Config const& config) -> std::expected<void, std::string>;
    auto solve_center(MpcAxisTrajectory const& reference) -> std::expected<double, std::string>;
};

} // namespace rmcs::fire_control
