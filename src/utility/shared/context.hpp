#pragma once

#include "utility/clock.hpp"
#include "utility/math/linear.hpp"
#include "utility/robot/id.hpp"
#include <cmath>
#include <limits>

namespace rmcs {

enum class ShootMode {
    STOPPING,
    OUTPOST,
    BATTLE,
    BUFF_SMALL,
    BUFF_LARGE,
};

struct Transform {
    Translation position { };
    Orientation orientation { };
};

struct AutoAimState {
    static constexpr auto kLabel  = "/shm_autoaim_state";
    static constexpr auto kLength = 512;

    TimePoint timestamp { };

    bool gimbal_takeover { false };
    bool shoot_permitted = { false };

    double yaw { std::numeric_limits<double>::quiet_NaN() };
    double pitch { std::numeric_limits<double>::quiet_NaN() };

    DeviceId target { DeviceId::UNKNOWN };

    auto reset() noexcept -> void {
        timestamp = Clock::now();

        gimbal_takeover = false;
        shoot_permitted = false;
        yaw             = std::numeric_limits<double>::quiet_NaN();
        pitch           = std::numeric_limits<double>::quiet_NaN();
        target          = DeviceId::UNKNOWN;
    }

    auto set_hold_state(double current_yaw, double current_pitch, DeviceId current_target) noexcept
        -> void {
        timestamp = Clock::now();

        gimbal_takeover = true;
        shoot_permitted = false;
        yaw             = current_yaw;
        pitch           = current_pitch;
        target          = current_target;
    }

    auto set_tracking_state(double target_yaw, double target_pitch, DeviceId tracked_target,
        bool allow_shoot) noexcept -> void {
        timestamp = Clock::now();

        gimbal_takeover = true;
        shoot_permitted = allow_shoot;
        yaw             = target_yaw;
        pitch           = target_pitch;
        target          = tracked_target;
    }

    [[nodiscard]] auto has_control_direction() const noexcept -> bool {
        return gimbal_takeover && std::isfinite(yaw) && std::isfinite(pitch);
    }
};
static_assert(std::is_trivially_copyable_v<AutoAimState>);

struct ControlState {
    static constexpr auto kLabel  = "/shm_control_state";
    static constexpr auto kLength = 512;

    /// Dynamic Context
    ///
    TimePoint timestamp { };
    ShootMode shoot_mode { ShootMode::BATTLE };

    double yaw { std::numeric_limits<double>::quiet_NaN() };
    double pitch { std::numeric_limits<double>::quiet_NaN() };

    Transform odom_to_camera_transform { };

    /// Lazy Context
    ///
    DeviceIds invincible_devices { DeviceIds::None() };

    auto reset() noexcept -> void {
        timestamp                = Clock::now();
        shoot_mode               = ShootMode::STOPPING;
        yaw                      = std::numeric_limits<double>::quiet_NaN();
        pitch                    = std::numeric_limits<double>::quiet_NaN();
        invincible_devices       = DeviceIds::None();
        odom_to_camera_transform = { };
    }
};
static_assert(std::is_trivially_copyable_v<ControlState>);
} // namespace rmcs
