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
    auto fetch() noexcept -> std::optional<StateType> {
        auto& client = get_client<StateType>();

        if constexpr (requires { client.is_updated(); }) {
            // Note:如果数据没有updated则返回上次收到的数据
            if (!ensure_open(client, util::shm_name<StateType>)) {
                return std::nullopt;
            }
            StateType out {};
            client.with_read([&](const StateType& data) { out = data; });
            return out;
        } else {
            static_assert(sizeof(StateType) == 0, "Error: This side can only WRITE this state.");
        }
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
