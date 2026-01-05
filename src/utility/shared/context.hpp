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

struct Transform {
    Translation position {};
    Orientation orientation {};
};

struct AutoAimState {
    Stamp timestamp {};

    bool should_control { false };
    bool should_shoot = { false };

    double yaw { 0 };
    double pitch { 0 };

    DeviceId target { DeviceId::UNKNOWN };
};
static_assert(std::is_trivially_copyable_v<AutoAimState>);

struct ControlState {
    Stamp timestamp {};
    ShootMode shoot_mode { ShootMode::BATTLE };

    double bullet_speed {};

    DeviceIds invincible_devices { DeviceIds::None() };
    /*Note:
     * 对应关系：
     * odom<->fast_tf::OdomImu,
     * camera<->fast_tf::CameraLink
     * */
    Transform camera_to_odom_transform {};
};
static_assert(std::is_trivially_copyable_v<ControlState>);
}
