#pragma once
#include "module/predictor/regular/ekf_parameter.hpp"
#include "module/predictor/snapshot.hpp"
#include "utility/pimpl.hpp"

#include <span>

namespace rmcs::predictor {

class RegularRobotState {
    RMCS_PIMPL_DEFINITION(RegularRobotState)

public:
    using EKF = EKFParameters::EKF;

    explicit RegularRobotState(TimePoint stamp) noexcept;
    RegularRobotState(RegularRobotState&&) noexcept;

    auto operator=(RegularRobotState&&) noexcept -> RegularRobotState&;

    auto initialize(Armor3D const& armor, TimePoint t) -> void;
    auto predict(TimePoint t) -> void;

    auto update(std::span<Armor3D const> armors) -> bool;

    auto is_converged() const -> bool;
    auto get_snapshot() const -> Snapshot;
    auto distance() const -> double;
};

} // namespace rmcs::predictor
