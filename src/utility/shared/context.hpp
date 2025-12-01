#pragma once
#include "utility/shared/interprocess.hpp"
#include <chrono>

namespace rmcs::util {

constexpr auto shared_memory_id { "/rmcs_auto_aim" };

using Clock = std::chrono::steady_clock;
using Stamp = Clock::time_point;

struct AutoAimState {
    Stamp timestamp;
};
static_assert(std::is_trivially_copyable_v<AutoAimState>);

struct ControlState {
    Stamp timestamp;
};
static_assert(std::is_trivially_copyable_v<ControlState>);

struct AutoAimClient {
    using Send = shm::Client<AutoAimState>::Send;
    using Recv = shm::Client<ControlState>::Recv;
};

struct ControlClient {
    using Send = shm::Client<ControlState>::Send;
    using Recv = shm::Client<AutoAimState>::Recv;
};

}
