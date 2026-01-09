#pragma once

#include <chrono>

#include "module/predictor/snapshot.hpp"
#include "utility/clock.hpp"
#include "utility/pimpl.hpp"

namespace rmcs::predictor {
struct RobotState {
    using Clock = util::Clock;

    RMCS_PIMPL_DEFINITION(RobotState)
public:
    struct MatchResult {
        int armor_id;
        double error;
        bool is_valid;
    };

    auto initialize(Armor3D const&, Clock::time_point) -> void;

    auto predict(Clock::time_point t) -> void;

    auto match(Armor3D const& armor) const -> MatchResult;
    auto update(Armor3D const& armor) -> void;

    auto is_converged() const -> bool;

    auto get_snapshot() const -> Snapshot;

    auto distance() const -> double;
};
}
