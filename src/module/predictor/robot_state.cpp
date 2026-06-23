#include "robot_state.hpp"

#include "module/predictor/outpost/robot_state.hpp"
#include "module/predictor/regular/robot_state.hpp"
#include "utility/time.hpp"

#include <limits>
#include <optional>
#include <variant>

using namespace rmcs::predictor;

struct RobotState::Impl {
    using State = std::variant<RegularRobotState, OutpostRobotState>;

    std::optional<State> state;
    std::optional<TimePoint> time_stamp;

    auto emplace_state(DeviceId device) -> State& {
        switch (device) {
        case DeviceId::OUTPOST:
            return state.emplace(std::in_place_type<OutpostRobotState>);
        default:
            return state.emplace(std::in_place_type<RegularRobotState>);
        }
    }

    auto predict(TimePoint t) -> void {
        if (!time_stamp.has_value()) {
            time_stamp = t;
            return;
        }

        const auto dt = util::delta_time(t, *time_stamp).count();
        time_stamp    = t;

        if (!state) return;
        std::visit([dt](auto& model) { model.predict(dt); }, *state);
    }

    auto update(std::span<Armor3d const> armors) -> bool {
        if (armors.empty()) return false;
        auto& target_state = state ? *state : emplace_state(armors.front().genre);
        return std::visit([armors](auto& model) { return model.update(armors); }, target_state);
    }

    auto is_converged() const -> bool {
        if (!state) return false;
        return std::visit([](auto const& model) { return model.is_converged(); }, *state);
    }

    auto get_snapshot() const -> std::optional<Snapshot> {
        if (!state || !time_stamp.has_value()) return std::nullopt;
        return std::visit(
            [this](auto const& model) { return model.get_snapshot(*time_stamp); }, *state);
    }

    auto distance() const -> double {
        if (!state) return std::numeric_limits<double>::infinity();
        return std::visit([](auto const& model) { return model.distance(); }, *state);
    }
};

RobotState::RobotState() noexcept
    : pimpl { std::make_unique<Impl>() } { }
RobotState::~RobotState() noexcept = default;

auto RobotState::predict(TimePoint t) -> void { return pimpl->predict(t); }

auto RobotState::update(std::span<Armor3d const> armors) -> bool { return pimpl->update(armors); }

auto RobotState::is_converged() const -> bool { return pimpl->is_converged(); }

auto RobotState::get_snapshot() const -> std::optional<Snapshot> { return pimpl->get_snapshot(); }

auto RobotState::distance() const -> double { return pimpl->distance(); }
