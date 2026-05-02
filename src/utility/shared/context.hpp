#pragma once

#include "utility/clock.hpp"
#include "utility/math/linear.hpp"
#include "utility/robot/id.hpp"

namespace rmcs {

template <class T>
concept context_trait = std::is_trivially_copyable_v<T>;

struct AutoAimState {
    static constexpr auto kLabel  = "/shm_autoaim_state";
    static constexpr auto kLength = 512;

    TimePoint timestamp { };

    bool should_control { false };
    bool should_shoot = { false };

    double yaw { std::numeric_limits<double>::quiet_NaN() };
    double pitch { std::numeric_limits<double>::quiet_NaN() };
    double yaw_rate { std::numeric_limits<double>::quiet_NaN() };
    double pitch_rate { std::numeric_limits<double>::quiet_NaN() };
    double yaw_acc { std::numeric_limits<double>::quiet_NaN() };
    double pitch_acc { std::numeric_limits<double>::quiet_NaN() };
    bool feedforward_valid { false };

    Translation robot_center {
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
    };

    DeviceId target { DeviceId::UNKNOWN };

    static auto kInvalid() {
        return AutoAimState {
            .timestamp = Clock::now(),
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

    bool enable_autoaim = false;

    double yaw { std::numeric_limits<double>::quiet_NaN() };
    double pitch { std::numeric_limits<double>::quiet_NaN() };

    Transform camera_transform = Transform::kNaN(); // Imu Odom Link

    /// Lazy Context
    ///
    DeviceIds invincible_devices = DeviceIds::None();

    static auto kInvalid() {
        return SystemContext {
            .timestamp = Clock::now(),
        };
    }
    static auto kIdentity() {
        return SystemContext {
            .timestamp        = Clock::now(),
            .enable_autoaim   = true,
            .yaw              = 0,
            .pitch            = 0,
            .camera_transform = Transform::kIdentity(),
        };
    }
};
static_assert(context_trait<SystemContext>);

} // namespace rmcs
