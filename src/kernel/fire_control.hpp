#pragma once

#include <eigen3/Eigen/Geometry>
#include <expected>
#include <yaml-cpp/yaml.h>

#include "module/predictor/snapshot.hpp"
#include "utility/clock.hpp"
#include "utility/pimpl.hpp"

namespace rmcs::kernel {

class FireControl {
    using Clock = util::Clock;

    RMCS_PIMPL_DEFINITION(FireControl)

public:
    struct Result {
        double pitch;
        double yaw;
        double horizon_distance;
    };

    auto initialize(const YAML::Node&) noexcept -> std::expected<void, std::string>;

    auto solve(const predictor::Snapshot& snapshot,
        Eigen::Vector3d const& muzzle_to_odom_translation) -> std::optional<Result>;
};
}
