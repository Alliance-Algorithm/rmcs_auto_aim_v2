#pragma once

#include <eigen3/Eigen/Core>
#include <expected>
#include <optional>

#include <yaml-cpp/yaml.h>

#include "module/fire_control/types.hpp"
#include "module/predictor/snapshot.hpp"
#include "utility/pimpl.hpp"

namespace rmcs::kernel {

class FireControl {
    RMCS_PIMPL_DEFINITION(FireControl)

public:
    struct Feedforward {
        double yaw;
        double pitch;
        double pitch_rate;
        double yaw_rate;
        double pitch_acc;
        double yaw_acc;
    };

    struct Result {
        double pitch;
        double yaw;
        std::optional<Feedforward> feedforward;
        bool shoot_permitted;
        Eigen::Vector3d center_position;
        TimePoint impact_time;
    };

    auto initialize(const YAML::Node&) noexcept -> std::expected<void, std::string>;

    auto solve(const predictor::Snapshot& snapshot, fire_control::GimbalState const& gimbal_state)
        -> std::optional<Result>;
};

} // namespace rmcs::kernel
