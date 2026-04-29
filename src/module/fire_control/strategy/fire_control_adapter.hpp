#pragma once

#include "module/fire_control/strategy/aim_point_provider.hpp"
#include "module/predictor/snapshot.hpp"

namespace rmcs::fire_control {

enum class GateMode : bool {
    DEFAULT,
    FACING,
};

struct PolicyDecision {
    AimPointProvider::Mode aim_mode { AimPointProvider::Mode::ARMOR };
    GateMode gate_mode { GateMode::DEFAULT };
};

class FireControlAdapter {
public:
    auto initialize(bool is_lazy_gimbal) noexcept -> void;

    auto resolve(predictor::Snapshot const& snapshot) const noexcept -> PolicyDecision;

private:
    bool is_lazy_gimbal_ { false };
};

} // namespace rmcs::fire_control
