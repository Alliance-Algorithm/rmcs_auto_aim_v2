#pragma once

#include <eigen3/Eigen/Dense>
#include <expected>
#include <functional>
#include <optional>
#include <yaml-cpp/yaml.h>

#include "utility/clock.hpp"
#include "utility/pimpl.hpp"

namespace rmcs::fire_control {

class MpcTrajectoryPlanner {
    RMCS_PIMPL_DEFINITION(MpcTrajectoryPlanner)

public:
    using AimPointSampler = std::function<std::optional<Eigen::Vector3d>(TimePoint)>;

    struct Plan {
        // 未约束的理想值
        double target_yaw { 0.0 };
        double target_pitch { 0.0 };
        // 有约束的最优值
        double yaw { 0.0 };
        double pitch { 0.0 };
    };

    auto configure_yaml(const YAML::Node& yaml) noexcept -> std::expected<void, std::string>;

    auto plan(TimePoint center_time, double bullet_speed, double yaw_offset, double pitch_offset,
        AimPointSampler const& sample_aim_point) -> std::optional<Plan>;
};

} // namespace rmcs::fire_control
