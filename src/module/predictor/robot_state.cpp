#include "robot_state.hpp"

#include "module/predictor/detail/robot_state_backend.hpp"

using namespace rmcs::predictor;

namespace {

auto empty_snapshot(RobotState::Clock::time_point stamp) -> Snapshot {
    return { Snapshot::NormalEKF::XVec::Zero(), rmcs::DeviceId::UNKNOWN, rmcs::CampColor::UNKNOWN,
        0, stamp };
}

} // namespace

struct RobotState::Impl {
    std::unique_ptr<detail::IRobotStateBackend> backend {};
    Clock::time_point pending_time_stamp { Clock::now() };

    [[nodiscard]] static auto make_backend(DeviceId device, Clock::time_point stamp)
        -> std::unique_ptr<detail::IRobotStateBackend> {
        auto const kind = detail::classify_robot_state_backend(device);
        return detail::make_robot_state_backend(kind, stamp);
    }

    auto reset_backend(Armor3D const& armor, Clock::time_point stamp) -> void {
        backend            = make_backend(armor.genre, stamp);
        pending_time_stamp = stamp;
    }

    auto ensure_backend(Armor3D const& armor) -> void {
        if (backend) return;
        backend = make_backend(armor.genre, pending_time_stamp);
    }

    auto initialize(Armor3D const& armor, Clock::time_point t) -> void {
        reset_backend(armor, t);
        backend->initialize(armor, t);
    }

    auto predict(Clock::time_point t) -> void {
        pending_time_stamp = t;
        if (backend) backend->predict(t);
    }

    auto update(std::span<Armor3D const> armors) -> bool {
        if (armors.empty()) return false;
        ensure_backend(armors.front());
        return backend ? backend->update(armors) : false;
    }

    auto is_converged() const -> bool { return backend ? backend->is_converged() : false; }

    auto get_snapshot() const -> Snapshot {
        return backend ? backend->get_snapshot() : empty_snapshot(pending_time_stamp);
    }

    auto distance() const -> double { return backend ? backend->distance() : 0.0; }
};

RobotState::RobotState() noexcept
    : pimpl { std::make_unique<Impl>() } { }
RobotState::~RobotState() noexcept = default;

auto RobotState::initialize(rmcs::Armor3D const& armor, Clock::time_point t) -> void {
    return pimpl->initialize(armor, t);
}

auto RobotState::predict(Clock::time_point t) -> void { return pimpl->predict(t); }

auto RobotState::update(std::span<Armor3D const> armors) -> bool { return pimpl->update(armors); }

auto RobotState::is_converged() const -> bool { return pimpl->is_converged(); }

auto RobotState::get_snapshot() const -> Snapshot { return pimpl->get_snapshot(); }

auto RobotState::distance() const -> double { return pimpl->distance(); }
