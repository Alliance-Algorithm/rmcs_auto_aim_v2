#pragma once

#include <eigen3/Eigen/Core>

namespace rmcs::fire_control {

inline constexpr int kMpcAxisHorizon = 100;

using MpcAxisTrajectory = Eigen::Matrix<double, 2, kMpcAxisHorizon>;
using ReferenceTrajectory = Eigen::Matrix<double, 4, kMpcAxisHorizon>;

} // namespace rmcs::fire_control
