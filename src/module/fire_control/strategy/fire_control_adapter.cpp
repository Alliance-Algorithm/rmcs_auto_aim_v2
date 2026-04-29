#include "module/fire_control/strategy/fire_control_adapter.hpp"

using namespace rmcs::fire_control;

auto FireControlAdapter::initialize(bool is_lazy_gimbal) noexcept -> void {
    is_lazy_gimbal_ = is_lazy_gimbal;
}

auto FireControlAdapter::resolve(predictor::Snapshot const& snapshot) const noexcept
    -> PolicyDecision {
    if (!is_lazy_gimbal_) {
        return {
            .aim_mode  = AimPointProvider::Mode::ARMOR,
            .gate_mode = GateMode::DEFAULT,
        };
    }

    if (snapshot.device_id() == DeviceId::OUTPOST) {
        return {
            .aim_mode  = AimPointProvider::Mode::ARMOR,
            .gate_mode = GateMode::DEFAULT,
        };
    }

    return {
        .aim_mode  = AimPointProvider::Mode::CENTER,
        .gate_mode = GateMode::FACING,
    };
}
