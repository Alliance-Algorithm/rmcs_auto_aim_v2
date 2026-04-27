#pragma once

#include "utility/clock.hpp"
#include "utility/math/linear.hpp"
#include "utility/robot/id.hpp"
#include <cmath>
#include <limits>

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

struct Transform {
    Translation position { };
    Orientation orientation { };

    static constexpr auto kNaN() {
        return Transform {
            Translation {
                std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN(),
            },
            Orientation {
                std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN(),
            },
        };
    };
    static constexpr auto kIdentity() {
        return Transform {
            Translation { 0, 0, 0 },
            Orientation { 0, 0, 0, 1 },
        };
    }
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

    static auto kInvalid() {
        return AutoAimState {
            .timestamp       = Clock::now(),
            .gimbal_takeover = false,
            .shoot_permitted = false,
            .yaw             = std::numeric_limits<double>::quiet_NaN(),
            .pitch           = std::numeric_limits<double>::quiet_NaN(),
            .target          = DeviceId::UNKNOWN,
        };
    }
};
static_assert(context_trait<AutoAimState>);

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

    struct {
        TimePoint timestamp = Clock::now();
        std::uint64_t index = 0;
    } capture_signals;

    /// Lazy Context
    ///
    DeviceIds invincible_devices { DeviceIds::None() };

    static auto kInvalid() {
        return ControlState {
            .timestamp                = Clock::now(),
            .shoot_mode               = ShootMode::STOPPING,
            .yaw                      = std::numeric_limits<double>::quiet_NaN(),
            .pitch                    = std::numeric_limits<double>::quiet_NaN(),
            .odom_to_camera_transform = Transform::kNaN(),
            .capture_signals          = { },
            .invincible_devices       = DeviceIds::None(),
        };
    }
    static auto kIdentity() {
        return ControlState {
            .timestamp                = Clock::now(),
            .shoot_mode               = ShootMode::BATTLE,
            .yaw                      = 0,
            .pitch                    = 0,
            .odom_to_camera_transform = Transform::kIdentity(),
            .capture_signals          = { },
            .invincible_devices       = DeviceIds::None(),
        };
    }
};
static_assert(context_trait<ControlState>);

} // namespace rmcs
