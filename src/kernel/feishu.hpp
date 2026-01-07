#pragma once

#include "utility/shared/context.hpp"
#include "utility/shared/interprocess.hpp"

namespace rmcs::kernel {
template <typename T>
constexpr const char* shm_name = nullptr;

template <>
constexpr auto shm_name<util::AutoAimState> = "/shm_autoaim_state";

template <>
constexpr auto shm_name<util::ControlState> = "/shm_control_state";

enum class RuntimeRole { AutoAim, Control };

template <RuntimeRole Role>
class Feishu {
public:
    using AutoAimState = util::AutoAimState;
    using ControlState = util::ControlState;

    using SendData = std::conditional_t<Role == RuntimeRole::AutoAim, AutoAimState, ControlState>;
    using RecvData = std::conditional_t<Role == RuntimeRole::AutoAim, ControlState, AutoAimState>;

    using SendClient = typename rmcs::shm::Client<SendData>::Send;
    using RecvClient = typename rmcs::shm::Client<RecvData>::Recv;

    Feishu() {
        send_client.open(shm_name<SendData>);
        recv_client.open(shm_name<RecvData>);
    }

    auto commit(SendData const& data) noexcept -> bool {
        if (!send_client.opened()) [[unlikely]]
            return false;
        send_client.with_write([&](SendData& shared) { shared = data; });
        return true;
    }

    auto fetch() noexcept -> const RecvData& {
        // Note:直接读取当前共享内存中的数据；如需检测是否有新数据，请先调用 updated()
        recv_client.with_read([&](RecvData const& shared) { recv_buffer = shared; });
        return recv_buffer;
    }

    auto updated() noexcept -> bool { return recv_client.opened() && recv_client.is_updated(); }

private:
    SendClient send_client {};
    RecvClient recv_client {};

    RecvData recv_buffer {};
};

}
