#pragma once

#include "utility/clock.hpp"
#include "utility/math/linear.hpp"
#include "utility/pimpl.hpp"
#include "utility/robot/id.hpp"

#include <rmcs_msgs/robot_id.hpp>

#include <atomic>
#include <deque>
#include <mutex>

namespace rmcs {

class AutoAim {
    RMCS_PIMPL_DEFINITION(AutoAim)

public:
    struct Command {
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
            return Command {
                .timestamp = Clock::now(),
            };
        }
    };

    struct Context {
        using RobotId = rmcs_msgs::RobotId;

        struct TransformFrame {
            Timestamp timestamp = { };
            Transform transform = Transform::kIdentity();

            double yaw   = kNaN;
            double pitch = kNaN;
        };
        std::deque<TransformFrame> transforms;

        DeviceIds invincible = DeviceIds::None();

        RobotId id = RobotId::UNKNOWN;
    };

    template <typename WithFunc>
        requires std::invocable<WithFunc, Context&>
    auto with_context(WithFunc&& func) {
        std::lock_guard lock(context_mutex);
        func(current_context);
    }

    template <typename WithFunc>
        requires std::invocable<WithFunc, const Command&>
    auto with_command(WithFunc&& func) {
        std::lock_guard lock(command_mutex);
        func(current_command);
    }

    auto command_updated() -> bool {
        return unread_command.exchange(false, std::memory_order::acquire);
    }

private:
    std::mutex context_mutex;
    Context current_context;

    std::atomic<bool> unread_command = false;
    std::mutex command_mutex;
    Command current_command;
};

}
