#pragma once

#include "module/tracker/trackable.hpp"
#include "utility/image/image.hpp"
#include "utility/math/linear.hpp"
#include "utility/pimpl.hpp"
#include "utility/robot/id.hpp"

#include <rmcs_msgs/robot_id.hpp>

#include <atomic>
#include <mutex>

namespace rmcs {

class AutoAim {
    RMCS_PIMPL_DEFINITION(AutoAim)

public:
    struct Command {
        Trackable::Unique trackable { };
    };

    struct Context {
        using RobotId = rmcs_msgs::RobotId;

        /// 相机在 PitchLink 下的平移外参，由 component 构造时写入一次
        Translation camera_translation = Translation::kZero();

        bool track_intent = false;
        bool track_rune   = false;

        double max_yaw_vel = 3.0;
        double max_yaw_acc = 100.0;

        DeviceIds track_ids = DeviceIds::Full();

        RobotId id = RobotId::UNKNOWN;

        struct Addition {
            Point3d attack = Point3d::kNaN();

            double aim_yaw   = kNaN;
            double aim_pitch = kNaN;

            bool should_track = false;
            bool should_shoot = false;
            bool pre_aim      = false;
        } addition;
    };

    auto process(const Image& image) -> void;

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
