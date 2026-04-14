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
    using Clock = util::Clock;
    using EKF   = OutpostEKFParameters::EKF;

    explicit OutpostRobotState(Clock::time_point stamp) noexcept;

    auto initialize(Armor3D const& armor, Clock::time_point t) -> void;
    auto predict(Clock::time_point t) -> void;

    auto update(Armor3D const& armor) -> bool;
    auto update(std::span<Armor3D const> armors) -> bool;

    auto is_converged() const -> bool;
    auto get_snapshot() const -> Snapshot;
    auto distance() const -> double;

    RMCS_PIMPL_DEFINITION(OutpostRobotState)
public:
    OutpostRobotState(OutpostRobotState&&) noexcept;
    auto operator=(OutpostRobotState&&) noexcept -> OutpostRobotState&;
};

} // namespace rmcs::predictor
