#pragma once

#include <chrono>
#include <span>

#include "module/predictor/outpost/ekf_parameter.hpp"
#include "module/predictor/snapshot.hpp"
#include "utility/clock.hpp"
#include "utility/pimpl.hpp"

namespace rmcs::predictor {

class OutpostRobotState {
public:
    using EKF = OutpostEKFParameters::EKF;

    explicit OutpostRobotState(TimePoint stamp) noexcept;

    auto initialize(Armor3D const& armor, TimePoint t) -> void;
    auto predict(TimePoint t) -> void;

    auto update(std::span<Armor3D const> armors) -> bool;

    auto is_converged() const -> bool;
    auto get_snapshot() const -> Snapshot;
    auto distance() const -> double;

    RMCS_PIMPL_DEFINITION(OutpostRobotState)
};

} // namespace rmcs::predictor
