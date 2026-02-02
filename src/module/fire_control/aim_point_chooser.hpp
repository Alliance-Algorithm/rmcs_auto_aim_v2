#pragma once

#include <eigen3/Eigen/Geometry>
#include <optional>
#include <span>
#include <yaml-cpp/yaml.h>

#include "utility/math/kalman_filter/ekf.hpp"
#include "utility/pimpl.hpp"
#include "utility/robot/armor.hpp"

namespace rmcs::fire_control {
class AimPointChooser {
private:
    using EKF = util::EKF<11, 4>;

public:
    struct Config {
        double coming_angle;               // rad
        double leaving_angle;              // rad
        double angular_velocity_threshold; // rad/s
        double outpost_coming_angle;       // rad
        double outpost_leaving_angle;      // rad
    };
    auto initialize(Config const& config) noexcept -> void;

    auto choose_armor(std::span<Armor3D const> armors, EKF::XVec const& ekf_x)
        -> std::optional<Armor3D>;

    RMCS_PIMPL_DEFINITION(AimPointChooser)
};

} // namespace rmcs::fire_control
