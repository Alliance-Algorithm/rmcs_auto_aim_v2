#if defined(__clang__)
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
#elif defined(__GNUC__)
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

#include "adapter/adapter.hpp"
#include "kernel/feishu.hpp"
#include "utility/rclcpp/node.hpp"
#include "utility/shared/context.hpp"

#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/robot_id.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs {

using namespace util;
using namespace kernel;

class AutoAimComponent final : public rmcs_executor::Component {
private:
    static constexpr auto kNaN          = std::numeric_limits<double>::quiet_NaN();
    static inline const auto kVectorNaN = Eigen::Vector3d { kNaN, kNaN, kNaN };

    Adapter adapter;
    RclcppNode rclcpp;
    Feishu<SystemContext, AutoAimState> feishu;

    OutputInterface<bool> should_control;
    OutputInterface<bool> should_shoot;
    OutputInterface<Eigen::Vector3d> target_direction;
    OutputInterface<Eigen::Vector3d> robot_center;
    OutputInterface<double> yaw_rate;
    OutputInterface<double> pitch_rate;
    OutputInterface<double> yaw_acc;
    OutputInterface<double> pitch_acc;
    OutputInterface<bool> feedforward_valid;

    InputInterface<rmcs_msgs::RobotId> robot_id;

    auto make_context() const {
        auto context = SystemContext {};

        context.timestamp = Clock::now();

        const auto dir = adapter.barrel_direction();
        context.yaw    = std::atan2(dir.y(), dir.x());
        context.pitch  = std::atan2(-dir.z(), std::hypot(dir.x(), dir.y()));

        const auto iso                       = adapter.camera_transform();
        context.camera_transform.translation = iso.translation();
        context.camera_transform.orientation = Eigen::Quaterniond(iso.rotation());

        // TODO:无敌状态下的装甲板需要从裁判系统获取并在此更新
        context.invincible_devices = DeviceIds::None();

        return context;
    }

public:
    explicit AutoAimComponent() noexcept
        : adapter { *this }
        , rclcpp { get_component_name() } {

        register_output("/auto_aim/should_control", should_control, false);
        register_output("/auto_aim/control_direction", target_direction, Eigen::Vector3d::Zero());
        register_output("/auto_aim/robot_center", robot_center, kVectorNaN);
        register_output("/auto_aim/should_shoot", should_shoot, false);
        register_output("/auto_aim/yaw_rate", yaw_rate, 0.0);
        register_output("/auto_aim/pitch_rate", pitch_rate, 0.0);
        register_output("/auto_aim/yaw_acc", yaw_acc, 0.0);
        register_output("/auto_aim/pitch_acc", pitch_acc, 0.0);
        register_output("/auto_aim/feedforward_valid", feedforward_valid, false);

        register_input("/referee/id", robot_id, true);
    }

    auto update() -> void override {
        // Reset all command
        {
            *should_control    = false;
            *should_shoot      = false;
            *target_direction  = kVectorNaN;
            *robot_center      = kVectorNaN;
            *yaw_rate          = kNaN;
            *pitch_rate        = kNaN;
            *yaw_acc           = kNaN;
            *pitch_acc         = kNaN;
            *feedforward_valid = false;
        }

        if (!adapter.ready()) [[unlikely]] {
            feishu.send(SystemContext::kInvalid());
            return;
        }

        feishu.send(make_context());

        if (!feishu.heartbeat()) return;

        // If heartbeat successfully, latest exists
        const auto command = *feishu.latest();

        // Timeout
        using namespace std::chrono_literals;
        if (Clock::now() - command.timestamp > 10ms) {
            return;
        }

        // 业务开关
        *should_control    = command.should_control;
        *should_shoot      = command.should_shoot;
        *yaw_rate          = command.yaw_rate;
        *pitch_rate        = command.pitch_rate;
        *yaw_acc           = command.yaw_acc;
        *pitch_acc         = command.pitch_acc;
        *feedforward_valid = command.feedforward_valid;
        *robot_center      = {
            command.robot_center.x,
            command.robot_center.y,
            command.robot_center.z,
        };

        if (!command.should_control) return;

        const auto pitch  = command.pitch;
        const auto yaw    = command.yaw;
        *target_direction = Eigen::Vector3d {
            std::cos(pitch) * std::cos(yaw),
            std::cos(pitch) * std::sin(yaw),
            std::sin(pitch),
        };
    }
};

} // namespace rmcs

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs::AutoAimComponent, rmcs_executor::Component)
