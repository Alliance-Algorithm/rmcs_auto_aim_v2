#pragma once

#include <expected>
#include <optional>
#include <string>

#include <eigen3/Eigen/Dense>

#include "module/fire_control/aim_point_chooser.hpp"
#include "module/predictor/snapshot.hpp"

namespace rmcs::fire_control {

struct AimAttitude {
    double yaw { 0.0 };
    double pitch { 0.0 };
    double fly_time { 0.0 };
};

class AimPointSampler {
public:
    static auto sample_attitude_at(predictor::Snapshot const& snapshot, AimPointChooser& chooser,
        TimePoint t, double bullet_speed) -> std::expected<AimAttitude, std::string>;

private:
    static auto sample_aim_point_at(predictor::Snapshot const& snapshot, AimPointChooser& chooser,
        TimePoint t) -> std::optional<Eigen::Vector3d>;

    static auto solve_aim_attitude(Eigen::Vector3d const& aim_point, double bullet_speed)
        -> std::expected<AimAttitude, std::string>;
};

} // namespace rmcs::fire_control
