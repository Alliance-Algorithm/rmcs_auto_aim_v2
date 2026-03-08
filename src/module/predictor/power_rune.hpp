#pragma once

#include "module/predictor/power_rune_snapshot.hpp"
#include "utility/pimpl.hpp"
#include "utility/robot/fan_blade.hpp"
#include "utility/robot/power_rune_mode.hpp"

namespace rmcs::predictor {
struct PowerRune {
    using Clock = util::Clock;
    using Snapshot = PowerRuneSnapshot;

    RMCS_PIMPL_DEFINITION(PowerRune)
public:
    auto initialize(FanBlade3D const&, PowerRuneMode mode, int clockwise, Clock::time_point t)
        -> void;

    auto predict(Clock::time_point t) -> void;
    auto update(FanBlade3D const& blade) -> void;
    auto reset() -> void;

    auto get_snapshot() const -> Snapshot;
};
} // namespace rmcs::predictor
