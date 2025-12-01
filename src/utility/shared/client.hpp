#pragma once
#include "utility/shared/context.hpp"
#include "utility/shared/interprocess.hpp"

namespace rmcs::util {

constexpr auto shared_autoaim_state_name { "/rmcs_autoaim_state" };
constexpr auto shared_control_state_name { "/rmcs_control_state" };

struct AutoAimClient {
    using Send = shm::Client<AutoAimState>::Send;
    using Recv = shm::Client<ControlState>::Recv;
};

struct ControlClient {
    using Send = shm::Client<ControlState>::Send;
    using Recv = shm::Client<AutoAimState>::Recv;
};

}
