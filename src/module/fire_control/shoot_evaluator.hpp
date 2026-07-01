#pragma once

#include "utility/pimpl.hpp"
#include <eigen3/Eigen/Core>
#include <limits>

namespace rmcs::fire_control {

class ShootEvaluator {
    RMCS_PIMPL_DEFINITION(ShootEvaluator)

public:
    struct Config {
        double yaw_tolerance { 0.07 };
        double pitch_tolerance { 0.04 };
        bool require_stable_command { true };
        bool is_lazy_gimbal { false };
    };

    struct Command {
        double yaw { std::numeric_limits<double>::quiet_NaN() };
        double pitch { std::numeric_limits<double>::quiet_NaN() };
        Eigen::Vector3d center { Eigen::Vector3d::Zero() };
        Eigen::Vector3d attack { Eigen::Vector3d::Zero() };
    };

    explicit ShootEvaluator(const Config& config);

    auto evaluate(Command const& command, double yaw, double pitch) noexcept -> bool;
};

} // namespace rmcs::fire_control
