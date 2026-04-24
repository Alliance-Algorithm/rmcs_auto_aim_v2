#pragma once

#include <expected>
#include <yaml-cpp/yaml.h>

#include "module/predictor/snapshot.hpp"
#include "utility/clock.hpp"
#include "utility/pimpl.hpp"

namespace rmcs::kernel {

class FireControl {
    RMCS_PIMPL_DEFINITION(FireControl)

public:
    struct Result {
        double pitch;
        double yaw;
        double horizon_distance;
        bool shoot_permitted;
    };

    auto initialize(const YAML::Node&) noexcept -> std::expected<void, std::string>;

    auto solve(const predictor::Snapshot& snapshot, bool control, double current_yaw)
        -> std::optional<Result>;
};
}
