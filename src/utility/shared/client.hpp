#pragma once
#include "utility/shared/context.hpp"
#include "utility/shared/interprocess.hpp"

namespace rmcs::util {

struct AutoAimSide { };
struct ControlSide { };

template <typename T>
concept IPCSide = std::same_as<T, AutoAimSide> || std::same_as<T, ControlSide>;

template <IPCSide Side, typename Data>
struct ShmRoleSelector {
    static constexpr bool is_sender =
        (std::same_as<Side, AutoAimSide> && std::same_as<Data, AutoAimState>)
        || (std::same_as<Side, ControlSide> && std::same_as<Data, ControlState>);

    using ClientType = std::conditional_t<is_sender, typename shm::Client<Data>::Send,
        typename shm::Client<Data>::Recv>;
};

template <IPCSide Side, typename DataType>
using ShmClient = typename ShmRoleSelector<Side, DataType>::ClientType;

template <typename T>
constexpr const char* shm_name = nullptr;

template <>
constexpr auto shm_name<AutoAimState> = "/shm_autoaim_state";

template <>
constexpr auto shm_name<ControlState> = "/shm_control_state";

}
