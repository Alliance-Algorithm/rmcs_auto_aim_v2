#pragma once
#include "utility/math/linear.hpp"
#include "utility/robot/id.hpp"
#include <chrono>

namespace rmcs::util {

using Clock = std::chrono::steady_clock;
using Stamp = Clock::time_point;

enum class ShootMode {
    STOPPING,
    OUTPOST,
    BATTLE,
    BUFF_SMALL,
    BUFF_LARGE,
};

struct AutoAimState {
    Stamp timestamp {};

    bool should_control = false;
    bool should_shoot   = false;

    Direction3d target_posture {};
    Orientation angular_speed {};
};
static_assert(std::is_trivially_copyable_v<AutoAimState>);

struct ControlState {
    Stamp timestamp {};
    ShootMode shoot_mode { ShootMode::BATTLE };

    double bullet_speed {};
    Orientation imu_state {};

    DeviceIds targets { DeviceIds::Full() };
};
static_assert(std::is_trivially_copyable_v<ControlState>);

}
