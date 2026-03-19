#pragma once

#include <expected>
#include <yaml-cpp/yaml.h>

#include "module/predictor/snapshot.hpp"
#include "utility/clock.hpp"
#include "utility/shared/context.hpp"
#include "utility/math/linear.hpp"
#include "utility/pimpl.hpp"

#include "vc/feature/rune_tracker.h"

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

    auto set_bullet_speed(double speed) -> void;

    auto solve(const predictor::Snapshot& snapshot, Translation const& odom_to_muzzle_translation)
        -> std::optional<Result>;

    auto solve_buff(
        std::shared_ptr<RuneTracker> target_tracker,
        const util::ControlState& control_state) -> std::optional<Result>;
};
}
