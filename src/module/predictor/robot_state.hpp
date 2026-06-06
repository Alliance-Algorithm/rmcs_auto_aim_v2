#pragma once
#include "module/predictor/snapshot.hpp"
#include "utility/clock.hpp"
#include "utility/pimpl.hpp"

#include <span>

namespace rmcs::predictor {

struct RobotState {
    RMCS_PIMPL_DEFINITION(RobotState)

public:
    auto initialize(Armor3d const&, TimePoint) -> void;

    auto predict(TimePoint t) -> void;

    auto update(std::span<Armor3d const> armors) -> bool;

    auto is_converged() const -> bool;

    auto get_snapshot() const -> Snapshot;

    auto distance() const -> double;
};

}
