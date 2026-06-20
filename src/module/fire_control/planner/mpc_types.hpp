#pragma once

#include <eigen3/Eigen/Core>

namespace rmcs::fire_control {

inline constexpr int kMpcHorizon = 50;
inline constexpr double kMpcDt   = 0.01;

using MpcAxisTrajectory   = Eigen::Matrix<double, 2, kMpcHorizon>;
using ReferenceTrajectory = Eigen::Matrix<double, 4, kMpcHorizon>;

} // namespace rmcs::fire_control
