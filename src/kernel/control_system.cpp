#include "control_system.hpp"
#include "utility/shared/client.hpp"

using namespace rmcs::kernel;
using namespace rmcs::util;

struct ControlSystem::Impl {
    AutoAimClient::Send shm_send {};
    AutoAimClient::Recv shm_recv {};

    ControlState control_state {};

    /// Send

    template <std::invocable<AutoAimState&> F>
    auto update_command(F&& f) noexcept {
        if (shm_send.opened() == false) {
            shm_send.open(util::shared_autoaim_state_name);
        }
        shm_send.with_write(std::forward<F>(f));
    }

    auto update_state(const AutoAimState& state) noexcept {
        if (shm_send.opened() == false) {
            shm_send.open(util::shared_autoaim_state_name);
        }
        shm_send.send(state);
    }

    /// Recv

    auto updated() const noexcept { return shm_recv.is_updated(); }

    auto system_state() noexcept -> const ControlState& {
        if (shm_recv.opened() == false) {
            shm_recv.open(util::shared_control_state_name);
        }
        shm_recv.recv(control_state);
        return control_state;
    }
};

auto ControlSystem::update_state(const AutoAimState& state) noexcept -> void {
    pimpl->update_state(state);
}

auto ControlSystem::updated() const noexcept -> bool { return pimpl->updated(); }

auto ControlSystem::system_state() const noexcept -> const ControlState& {
    return pimpl->system_state();
}

ControlSystem::ControlSystem() noexcept
    : pimpl { std::make_unique<Impl>() } { }

ControlSystem::~ControlSystem() noexcept = default;
