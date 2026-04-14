#pragma once

#include <chrono>
#include <span>

#include "module/predictor/ekf_parameter.hpp"
#include "module/predictor/snapshot.hpp"
#include "utility/pimpl.hpp"

namespace rmcs::predictor {

class RegularRobotState {
public:
    using Clock = util::Clock;
    using EKF   = EKFParameters::EKF;

    explicit RegularRobotState(Clock::time_point stamp) noexcept;

    auto initialize(Armor3D const& armor, Clock::time_point t) -> void;
    auto predict(Clock::time_point t) -> void;

    auto update(std::span<Armor3D const> armors) -> bool;

    auto is_converged() const -> bool;
    auto get_snapshot() const -> Snapshot;
    auto distance() const -> double;

    RMCS_PIMPL_DEFINITION(RegularRobotState)
public:
    RegularRobotState(RegularRobotState&&) noexcept;
    auto operator=(RegularRobotState&&) noexcept -> RegularRobotState&;
};

} // namespace rmcs::predictor
