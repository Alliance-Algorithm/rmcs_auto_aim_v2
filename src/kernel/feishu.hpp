#pragma once

#include "utility/shared/client.hpp"
namespace rmcs::kernel {

template <util::IPCSide Side>
class Feishu {
public:
    using AutoAimState = util::AutoAimState;
    using ControlState = util::ControlState;

    template <typename StateType>
    auto commit(StateType const& state) noexcept -> bool
        requires(util::ShmRoleSelector<Side, StateType>::is_sender)
    {
        auto& client = this->template get_client<StateType>();

        if constexpr (requires { client.with_write([](StateType&) { }); }) {
            if (!ensure_open(client, util::shm_name<StateType>)) [[unlikely]] {
                return false;
            }

            client.with_write([&](StateType& data) { data = state; });
            return true;
        } else {
            static_assert(sizeof(StateType) == 0, "Error: This side can only READ this state.");
        }
    }

    template <typename StateType>
    auto fetch() noexcept -> std::optional<StateType>
        requires(!util::ShmRoleSelector<Side, StateType>::is_sender)
    {
        auto& client = get_client<StateType>();

        // Note:直接读取当前共享内存中的数据；如需检测是否有新数据，请先调用 updated()
        if (!ensure_open(client, util::shm_name<StateType>)) {
            return std::nullopt;
        }
        auto out = StateType {};
        client.with_read([&](const StateType& data) { out = data; });
        return out;
    }

    template <typename StateType>
    auto updated() noexcept -> bool
        requires(!util::ShmRoleSelector<Side, StateType>::is_sender)
    {
        auto& client = get_client<StateType>();
        return ensure_open(client, util::shm_name<StateType>) && client.is_updated();
    }

private:
    util::ShmClient<Side, AutoAimState> auto_aim_client {};
    util::ShmClient<Side, ControlState> control_client {};

    template <typename DataType>
    auto get_client() noexcept -> auto& {
        if constexpr (std::same_as<DataType, AutoAimState>) {
            return auto_aim_client;
        } else {
            return control_client;
        }
    }

    template <typename Client>
    auto ensure_open(Client& client, const char* name) noexcept -> bool {
        return client.opened() || (name && client.open(name));
    }
};

}
