#pragma once

#include <eigen3/Eigen/Core>

namespace rmcs::fire_control {

inline constexpr int kMpcAxisHorizon = 100;

// row 0: angle (rad)
// row 1: angular velocity (rad/s)
using MpcAxisTrajectory = Eigen::Matrix<double, 2, kMpcAxisHorizon>;

// row 0: yaw angle (rad)
// row 1: yaw angular velocity (rad/s)
// row 2: pitch angle (rad)
// row 3: pitch angular velocity (rad/s)
using ReferenceTrajectory = Eigen::Matrix<double, 4, kMpcAxisHorizon>;

} // namespace rmcs::fire_control
