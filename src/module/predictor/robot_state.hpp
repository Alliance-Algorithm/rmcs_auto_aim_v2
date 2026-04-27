#pragma once

#include <span>

#include "module/predictor/snapshot.hpp"
#include "utility/clock.hpp"
#include "utility/pimpl.hpp"

#include <span>

namespace rmcs::predictor {

struct RobotState {
    RMCS_PIMPL_DEFINITION(RobotState)

public:
    auto initialize(Armor3D const&, TimePoint) -> void;

    auto predict(TimePoint t) -> void;

    auto update(std::span<Armor3D const> armors) -> bool;

    auto is_converged() const -> bool;

    auto get_snapshot() const -> Snapshot;

    auto distance() const -> double;
};

}
