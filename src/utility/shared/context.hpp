#pragma once

#include "utility/clock.hpp"
#include "utility/math/linear.hpp"
#include "utility/robot/id.hpp"

#include <rmcs_msgs/robot_id.hpp>

namespace rmcs {

template <class T>
concept context_trait = std::is_trivially_copyable_v<T>;

struct AutoAimState {
    static constexpr auto kLabel  = "/shm_autoaim_state";
    static constexpr auto kLength = 512;
    static constexpr auto kNaN    = std::numeric_limits<double>::quiet_NaN();

    TimePoint timestamp { };

    bool should_control { false };
    bool should_shoot = { false };

    double yaw { kNaN };
    double pitch { kNaN };
    double yaw_rate { kNaN };
    double pitch_rate { kNaN };
    double yaw_acc { kNaN };
    double pitch_acc { kNaN };
    bool feedforward_valid { false };

    Translation robot_center { kNaN, kNaN, kNaN };

    DeviceId target { DeviceId::UNKNOWN };

    static auto kInvalid() {
        return AutoAimState {
            .timestamp = Clock::now(),
        };
    }
};
static_assert(context_trait<AutoAimState>);

struct SystemContext {
    using RobotId = rmcs_msgs::RobotId;

    static constexpr auto kLabel  = "/shm_control_state";
    static constexpr auto kLength = 512;
    static constexpr auto kNaN    = std::numeric_limits<double>::quiet_NaN();

    /// Dynamic Context
    ///
    TimePoint timestamp { };

    bool enable_autoaim = false;

    double yaw { kNaN };
    double pitch { kNaN };

    Transform camera_transform = Transform::kNaN(); // Imu Odom Link

    /// Lazy Context
    ///
    DeviceIds invincible_devices = DeviceIds::None();

    RobotId id = RobotId::UNKNOWN;

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
