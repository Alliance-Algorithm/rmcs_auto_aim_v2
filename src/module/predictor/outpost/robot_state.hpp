#pragma once

#include "module/predictor/outpost/ekf_parameter.hpp"
#include "module/predictor/snapshot.hpp"
#include "utility/pimpl.hpp"

#include <optional>
#include <span>

namespace rmcs::predictor {

class OutpostRobotState {
    RMCS_PIMPL_DEFINITION(OutpostRobotState)

public:
    using EKF = OutpostEKFParameters::EKF;

    auto predict(double dt) -> void;

    auto update(std::span<Armor3d const> armors) -> bool;

    auto is_converged() const -> bool;
    auto get_snapshot(TimePoint stamp) const -> std::optional<Snapshot>;
    auto distance() const -> double;
};

} // namespace rmcs::predictor
