#pragma once

#include "utility/clock.hpp"
#include "utility/math/linear.hpp"
#include "utility/robot/id.hpp"

#include <rmcs_msgs/robot_id.hpp>

namespace rmcs {

struct AutoAimState {
    TimePoint timestamp { };

    bool should_track { false };
    bool should_shoot = { false };

    double yaw { kNaN };
    double pitch { kNaN };
    double yaw_rate { kNaN };
    double pitch_rate { kNaN };
    double yaw_acc { kNaN };
    double pitch_acc { kNaN };
    Translation robot_center { kNaN, kNaN, kNaN };

    DeviceId target { DeviceId::UNKNOWN };

    static auto kInvalid() {
        return AutoAimState {
            .timestamp = Clock::now(),
        };
    }
};

struct SystemContext {
    using RobotId = rmcs_msgs::RobotId;

    /// Dynamic Context
    ///
    TimePoint timestamp { };

    double yaw { kNaN };
    double pitch { kNaN };

    Transform camera_transform = Transform::kIdentity();

    /// Lazy Context
    ///
    DeviceIds invincible_devices = DeviceIds::None();

    RobotId id = RobotId::UNKNOWN;

    /// Template Context
    ///
    static auto kInvalid() {
        return SystemContext {
            .timestamp = Clock::now(),
        };
    }
    static auto kIdentity() {
        return SystemContext {
            .timestamp        = Clock::now(),
            .yaw              = 0,
            .pitch            = 0,
            .camera_transform = Transform::kIdentity(),
        };
    }
};

} // namespace rmcs
