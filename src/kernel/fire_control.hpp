#pragma once

#include <eigen3/Eigen/Core>
#include <expected>
#include <optional>
#include <yaml-cpp/yaml.h>

#include "module/predictor/snapshot.hpp"
#include "utility/pimpl.hpp"

namespace rmcs::kernel {

class FireControl {
    RMCS_PIMPL_DEFINITION(FireControl)

public:
    struct Result {
        double pitch;
        double yaw;
        double pitch_rate;
        double yaw_rate;
        double pitch_acc;
        double yaw_acc;
        bool feedforward_valid;
        bool shoot_permitted;
        Eigen::Vector3d center_position;
    };

    auto initialize(const YAML::Node&) noexcept -> std::expected<void, std::string>;

    auto solve(const predictor::Snapshot& snapshot, bool control, double current_yaw)
        -> std::optional<Result>;
};
}
