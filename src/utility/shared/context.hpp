#pragma once

#include "utility/clock.hpp"
#include "utility/math/linear.hpp"
#include "utility/robot/id.hpp"

namespace rmcs::util {

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
    Clock::time_point timestamp {};

    bool should_control { false };
    bool should_shoot = { false };

    double yaw { 0 };
    double pitch { 0 };

    DeviceId target { DeviceId::UNKNOWN };
};
static_assert(std::is_trivially_copyable_v<AutoAimState>);

struct ControlState {
    Clock::time_point timestamp {};
    ShootMode shoot_mode { ShootMode::BATTLE };

    double bullet_speed {};

    DeviceIds invincible_devices { DeviceIds::None() };

    Transform odom_to_camera_transform {};

    auto set_identity() noexcept -> void {
        timestamp                = Clock::now();
        shoot_mode               = ShootMode::STOPPING;
        bullet_speed             = 0.0;
        invincible_devices       = DeviceIds::None();
        odom_to_camera_transform = {};
    }
};
static_assert(std::is_trivially_copyable_v<ControlState>);
}
