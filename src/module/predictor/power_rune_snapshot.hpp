#pragma once

#include <memory>
#include <variant>

#include "module/predictor/buff_ekf_parameter.hpp"
#include "utility/clock.hpp"
#include "utility/robot/power_rune_mode.hpp"

namespace rmcs::predictor {
class PowerRuneSnapshot {
public:
    using Clock    = util::Clock;
    using XVariant = std::variant<std::monostate, SmallEKF::XVec, LargeEKF::XVec>;

    PowerRuneSnapshot() noexcept;
    PowerRuneSnapshot(
        XVariant ekf_x, PowerRuneModeOpt mode, int clockwise, Clock::time_point stamp) noexcept;
    PowerRuneSnapshot(PowerRuneSnapshot const&);
    PowerRuneSnapshot(PowerRuneSnapshot&&) noexcept;
    auto operator=(PowerRuneSnapshot const&) -> PowerRuneSnapshot&;
    auto operator=(PowerRuneSnapshot&&) noexcept -> PowerRuneSnapshot&;
    ~PowerRuneSnapshot() noexcept;

    auto ekf_x() const -> XVariant const&;
    auto mode() const -> PowerRuneModeOpt;
    auto clockwise() const -> int;
    auto time_stamp() const -> Clock::time_point;
    auto predict_at(Clock::time_point t) const -> XVariant;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};
} // namespace rmcs::predictor
