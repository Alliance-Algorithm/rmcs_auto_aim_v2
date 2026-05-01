#pragma once

#include "utility/clock.hpp"
#include "utility/math/linear.hpp"
#include "utility/robot/id.hpp"

namespace rmcs {

template <class T>
concept context_trait = std::is_trivially_copyable_v<T>;

enum class ShootMode {
    STOPPING,
    OUTPOST,
    BATTLE,
    BUFF_SMALL,
    BUFF_LARGE,
};

struct AutoAimState {
    static constexpr auto kLabel  = "/shm_autoaim_state";
    static constexpr auto kLength = 512;

    TimePoint timestamp { };

    bool should_control { false };
    bool should_shoot = { false };

    double yaw { std::numeric_limits<double>::quiet_NaN() };
    double pitch { std::numeric_limits<double>::quiet_NaN() };

    DeviceId target { DeviceId::UNKNOWN };

    static auto kInvalid() {
        return AutoAimState {
            .timestamp      = Clock::now(),
            .should_control = false,
            .should_shoot   = false,
            .yaw            = std::numeric_limits<double>::quiet_NaN(),
            .pitch          = std::numeric_limits<double>::quiet_NaN(),
            .target         = DeviceId::UNKNOWN,
        };
    }
};
static_assert(context_trait<AutoAimState>);

struct SystemContext {
    static constexpr auto kLabel  = "/shm_control_state";
    static constexpr auto kLength = 512;

    /// Dynamic Context
    ///
    TimePoint timestamp { };
    ShootMode shoot_mode { ShootMode::BATTLE };

    double yaw { std::numeric_limits<double>::quiet_NaN() };
    double pitch { std::numeric_limits<double>::quiet_NaN() };

    Transform camera_transform { }; // Imu Odom Link

    /// Lazy Context
    ///
    DeviceIds invincible_devices { DeviceIds::None() };

    static auto kInvalid() {
        return SystemContext {
            .timestamp          = Clock::now(),
            .shoot_mode         = ShootMode::STOPPING,
            .yaw                = std::numeric_limits<double>::quiet_NaN(),
            .pitch              = std::numeric_limits<double>::quiet_NaN(),
            .camera_transform   = Transform::kNaN(),
            .invincible_devices = DeviceIds::None(),
        };
    }
    static auto kIdentity() {
        return SystemContext {
            .timestamp          = Clock::now(),
            .shoot_mode         = ShootMode::BATTLE,
            .yaw                = 0,
            .pitch              = 0,
            .camera_transform   = Transform::kIdentity(),
            .invincible_devices = DeviceIds::None(),
        };
    }
};
static_assert(context_trait<SystemContext>);

} // namespace rmcs
