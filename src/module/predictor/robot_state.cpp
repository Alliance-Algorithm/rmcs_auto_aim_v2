#include "robot_state.hpp"

#include <variant>

#include "module/predictor/outpost_robot_state.hpp"
#include "module/predictor/regular_robot_state.hpp"

using namespace rmcs::predictor;

namespace {

template <class... Ts>
struct Overloaded : Ts... {
    using Ts::operator()...;
};

template <class... Ts>
Overloaded(Ts...) -> Overloaded<Ts...>;

auto invalid_match_result() -> RobotState::MatchResult { return { -1, 1e10, false }; }

auto empty_snapshot(RobotState::Clock::time_point stamp) -> Snapshot {
    return { Snapshot::NormalEKF::XVec::Zero(),
        rmcs::DeviceId::UNKNOWN,
        rmcs::CampColor::UNKNOWN,
        0,
        stamp };
}

} // namespace

struct RobotState::Impl {
    using Backend = std::variant<std::monostate, RegularRobotState, OutpostRobotState>;

    Backend backend {};
    Clock::time_point pending_time_stamp { Clock::now() };

    auto emplace_backend(DeviceId device, Clock::time_point stamp) -> void {
        if (device == DeviceId::OUTPOST)
            backend.template emplace<OutpostRobotState>(stamp);
        else
            backend.template emplace<RegularRobotState>(stamp);
    }

    auto reset_backend(Armor3D const& armor, Clock::time_point stamp) -> void {
        emplace_backend(armor.genre, stamp);
        pending_time_stamp = stamp;
    }

    auto ensure_backend(Armor3D const& armor) -> void {
        if (!std::holds_alternative<std::monostate>(backend)) return;
        emplace_backend(armor.genre, pending_time_stamp);
    }

    template <class Fn, class EmptyFn>
    decltype(auto) dispatch(Fn&& fn, EmptyFn&& empty_fn) {
        return std::visit(
            Overloaded {
                [&](std::monostate&) -> decltype(auto) { return empty_fn(); },
                [&](auto& state) -> decltype(auto) { return fn(state); },
            },
            backend);
    }

    template <class Fn, class EmptyFn>
    decltype(auto) dispatch(Fn&& fn, EmptyFn&& empty_fn) const {
        return std::visit(
            Overloaded {
                [&](std::monostate const&) -> decltype(auto) { return empty_fn(); },
                [&](auto const& state) -> decltype(auto) { return fn(state); },
            },
            backend);
    }

    auto initialize(Armor3D const& armor, Clock::time_point t) -> void {
        reset_backend(armor, t);
        dispatch([&](auto& state) { state.initialize(armor, t); }, [] {});
    }

    auto predict(Clock::time_point t) -> void {
        pending_time_stamp = t;
        dispatch([&](auto& state) { state.predict(t); }, [] {});
    }

    auto match(Armor3D const& armor) const -> MatchResult {
        return dispatch(
            [&](auto const& state) {
                auto result = state.match(armor);
                return MatchResult { result.armor_id, result.error, result.is_valid };
            },
            [] { return invalid_match_result(); });
    }

    auto update(Armor3D const& armor) -> bool {
        ensure_backend(armor);
        return dispatch([&](auto& state) { return state.update(armor); }, [] { return false; });
    }

    auto update(std::span<Armor3D const> armors) -> bool {
        if (armors.empty()) return false;
        ensure_backend(armors.front());
        return dispatch([&](auto& state) { return state.update(armors); }, [] { return false; });
    }

    auto is_converged() const -> bool {
        return dispatch([](auto const& state) { return state.is_converged(); }, [] { return false; });
    }

    auto get_snapshot() const -> Snapshot {
        return dispatch([](auto const& state) { return state.get_snapshot(); },
            [&] { return empty_snapshot(pending_time_stamp); });
    }

    auto distance() const -> double {
        return dispatch([](auto const& state) { return state.distance(); }, [] { return 0.0; });
    }
};

RobotState::RobotState() noexcept
    : pimpl { std::make_unique<Impl>() } { }
RobotState::~RobotState() noexcept = default;

auto RobotState::initialize(rmcs::Armor3D const& armor, Clock::time_point t) -> void {
    return pimpl->initialize(armor, t);
}

auto RobotState::predict(Clock::time_point t) -> void { return pimpl->predict(t); }

auto RobotState::match(Armor3D const& armor) const -> MatchResult { return pimpl->match(armor); }
auto RobotState::update(rmcs::Armor3D const& armor) -> bool { return pimpl->update(armor); }
auto RobotState::update(std::span<Armor3D const> armors) -> bool { return pimpl->update(armors); }

auto RobotState::is_converged() const -> bool { return pimpl->is_converged(); }

auto RobotState::get_snapshot() const -> Snapshot { return pimpl->get_snapshot(); }

auto RobotState::distance() const -> double { return pimpl->distance(); }
