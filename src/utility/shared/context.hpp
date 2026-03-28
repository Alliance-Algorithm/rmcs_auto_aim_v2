#pragma once

#include "utility/clock.hpp"
#include "utility/math/linear.hpp"
#include "utility/robot/id.hpp"
#include <cmath>
#include <limits>

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

    bool gimbal_takeover { false };
    bool shoot_permitted = { false };

    double yaw { std::numeric_limits<double>::quiet_NaN() };
    double pitch { std::numeric_limits<double>::quiet_NaN() };

    DeviceId target { DeviceId::UNKNOWN };

    auto set_identity() noexcept -> void {
        timestamp = Clock::now();

        gimbal_takeover = false;
        shoot_permitted = false;
        yaw             = std::numeric_limits<double>::quiet_NaN();
        pitch           = std::numeric_limits<double>::quiet_NaN();
        target          = DeviceId::UNKNOWN;
    }

    auto set_safe_state(double current_yaw, double current_pitch) noexcept -> void {
        timestamp = Clock::now();

        gimbal_takeover = false;
        shoot_permitted = false;
        yaw             = current_yaw;
        pitch           = current_pitch;
        target          = DeviceId::UNKNOWN;
    }
};
static_assert(std::is_trivially_copyable_v<AutoAimState>);

struct ControlState {
    Clock::time_point timestamp {};
    ShootMode shoot_mode { ShootMode::BATTLE };

    double yaw { std::numeric_limits<double>::quiet_NaN() };
    double pitch { std::numeric_limits<double>::quiet_NaN() };

    DeviceIds invincible_devices { DeviceIds::None() };

    Transform odom_to_camera_transform {};

    auto set_identity() noexcept -> void {
        timestamp                = Clock::now();
        shoot_mode               = ShootMode::STOPPING;
        yaw                      = std::numeric_limits<double>::quiet_NaN();
        pitch                    = std::numeric_limits<double>::quiet_NaN();
        invincible_devices       = DeviceIds::None();
        odom_to_camera_transform = {};
    }
};
static_assert(std::is_trivially_copyable_v<ControlState>);
}
