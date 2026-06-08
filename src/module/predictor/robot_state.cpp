#include "robot_state.hpp"

#include "module/predictor/outpost/robot_state.hpp"
#include "module/predictor/regular/robot_state.hpp"

#include <limits>
#include <optional>
#include <variant>

using namespace rmcs::predictor;

struct RobotState::Impl {
    using State = std::variant<RegularRobotState, OutpostRobotState>;

    std::optional<State> state;
    TimePoint pending_time_stamp { Clock::now() };

    auto emplace_state(DeviceId device, TimePoint stamp) -> State& {
        switch (device) {
        case DeviceId::OUTPOST:
            return state.emplace(std::in_place_type<OutpostRobotState>, stamp);
        default:
            return state.emplace(std::in_place_type<RegularRobotState>, stamp);
        }
    }

    auto initialize(Armor3d const& armor, TimePoint t) -> void {
        pending_time_stamp = t;
        auto& target_state = emplace_state(armor.genre, t);
        std::visit([&](auto& model) { model.initialize(armor, t); }, target_state);
    }

    auto predict(TimePoint t) -> void {
        pending_time_stamp = t;
        if (!state) return;
        std::visit([t](auto& model) { model.predict(t); }, *state);
    }

    auto update(std::span<Armor3d const> armors) -> bool {
        if (armors.empty()) return false;
        auto& target_state =
            state ? *state : emplace_state(armors.front().genre, pending_time_stamp);
        return std::visit([armors](auto& model) { return model.update(armors); }, target_state);
    }

    auto is_converged() const -> bool {
        if (!state) return false;
        return std::visit([](auto const& model) { return model.is_converged(); }, *state);
    }

    auto get_snapshot() const -> std::optional<Snapshot> {
        if (!state) return std::nullopt;
        return std::visit([](auto const& model) { return model.get_snapshot(); }, *state);
    }

    auto distance() const -> double {
        if (!state) return std::numeric_limits<double>::infinity();
        return std::visit([](auto const& model) { return model.distance(); }, *state);
    }
};

RobotState::RobotState() noexcept
    : pimpl { std::make_unique<Impl>() } { }
RobotState::~RobotState() noexcept = default;

auto RobotState::initialize(rmcs::Armor3d const& armor, TimePoint t) -> void {
    return pimpl->initialize(armor, t);
}

auto RobotState::predict(TimePoint t) -> void { return pimpl->predict(t); }

auto RobotState::update(std::span<Armor3d const> armors) -> bool { return pimpl->update(armors); }

auto RobotState::is_converged() const -> bool { return pimpl->is_converged(); }

auto RobotState::get_snapshot() const -> std::optional<Snapshot> { return pimpl->get_snapshot(); }

auto RobotState::distance() const -> double { return pimpl->distance(); }
