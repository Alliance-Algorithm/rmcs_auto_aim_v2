#pragma once

#include <chrono>
#include <span>

#include "module/predictor/snapshot.hpp"
#include "utility/clock.hpp"
#include "utility/pimpl.hpp"

namespace rmcs::predictor {
struct RobotState {
    using Clock = util::Clock;

    RMCS_PIMPL_DEFINITION(RobotState)
public:
    auto initialize(Armor3D const&, Clock::time_point) -> void;

    auto predict(Clock::time_point t) -> void;

    auto update(std::span<Armor3D const> armors) -> bool;

    auto is_converged() const -> bool;

    auto get_snapshot() const -> Snapshot;

    auto distance() const -> double;
};
}
