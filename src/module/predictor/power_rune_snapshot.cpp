#include "module/predictor/power_rune_snapshot.hpp"

#include <chrono>
#include <type_traits>
#include <utility>

#include "utility/time.hpp"

using namespace rmcs::predictor;

struct PowerRuneSnapshot::Impl {
    XVariant ekf_x { std::monostate {} };
    PowerRuneModeOpt mode { std::nullopt };
    int clockwise { 1 };
    Clock::time_point stamp {};

    Impl() noexcept = default;
    Impl(XVariant ekf_x, PowerRuneModeOpt mode, int clockwise, Clock::time_point stamp) noexcept
        : ekf_x { std::move(ekf_x) }
        , mode { mode }
        , clockwise { clockwise >= 0 ? 1 : -1 }
        , stamp { stamp } { }

    auto predict_at(Clock::time_point t) const -> XVariant {
        const auto dt = util::delta_time(t, stamp);
        if (dt <= std::chrono::duration<double>::zero()) {
            return ekf_x;
        }

        const auto dt_s = dt.count();
        return std::visit(
            [this, dt_s](auto const& x) -> XVariant {
                using T = std::decay_t<decltype(x)>;
                if constexpr (std::is_same_v<T, SmallEKF::XVec>) {
                    return BuffEKFParameters::rune_f<SmallEKF>(dt_s, clockwise)(x);
                } else if constexpr (std::is_same_v<T, LargeEKF::XVec>) {
                    return BuffEKFParameters::rune_f<LargeEKF>(dt_s, clockwise)(x);
                }
                return std::monostate {};
            },
            ekf_x);
    }
};

PowerRuneSnapshot::PowerRuneSnapshot() noexcept
    : pimpl { std::make_unique<Impl>() } { }

PowerRuneSnapshot::PowerRuneSnapshot(
    XVariant ekf_x, PowerRuneModeOpt mode, int clockwise, Clock::time_point stamp) noexcept
    : pimpl { std::make_unique<Impl>(std::move(ekf_x), mode, clockwise, stamp) } { }

PowerRuneSnapshot::PowerRuneSnapshot(PowerRuneSnapshot const& other)
    : pimpl { other.pimpl ? std::make_unique<Impl>(*other.pimpl) : std::make_unique<Impl>() } { }

PowerRuneSnapshot::PowerRuneSnapshot(PowerRuneSnapshot&&) noexcept = default;

auto PowerRuneSnapshot::operator=(PowerRuneSnapshot const& other) -> PowerRuneSnapshot& {
    if (this != &other) {
        pimpl = other.pimpl ? std::make_unique<Impl>(*other.pimpl) : std::make_unique<Impl>();
    }
    return *this;
}

auto PowerRuneSnapshot::operator=(PowerRuneSnapshot&&) noexcept -> PowerRuneSnapshot& = default;

PowerRuneSnapshot::~PowerRuneSnapshot() noexcept = default;

auto PowerRuneSnapshot::ekf_x() const -> XVariant const& {
    static const auto kDefaultX = XVariant { std::monostate {} };
    return pimpl ? pimpl->ekf_x : kDefaultX;
}
auto PowerRuneSnapshot::mode() const -> PowerRuneModeOpt {
    return pimpl ? pimpl->mode : std::nullopt;
}
auto PowerRuneSnapshot::clockwise() const -> int { return pimpl ? pimpl->clockwise : 1; }
auto PowerRuneSnapshot::time_stamp() const -> Clock::time_point {
    return pimpl ? pimpl->stamp : Clock::time_point {};
}
auto PowerRuneSnapshot::predict_at(Clock::time_point t) const -> XVariant {
    return pimpl ? pimpl->predict_at(t) : XVariant { std::monostate {} };
}
