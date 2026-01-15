#include "kernel/feishu.hpp"
#include "module/debug/action_throttler.hpp"
#include "module/debug/framerate.hpp"
#include "utility/rclcpp/node.hpp"
#include "utility/rclcpp/visual/transform.hpp"
#include "utility/shared/context.hpp"

#include <cmath>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs {

using namespace rmcs::util;
using namespace kernel;

class AutoAimComponent final : public rmcs_executor::Component {
public:
    explicit AutoAimComponent() noexcept
        : rclcpp { get_component_name() } {

        register_input("/tf", rmcs_tf);
        register_input("/gimbal/yaw/angle", current_gimbal_yaw);
        register_input("/gimbal/pitch/angle", current_gimbal_pitch);

        register_output("/gimbal/auto_aim/control_direction", target_direction);

        using namespace std::chrono_literals;
        framerate.set_interval(2s);

        visual::Transform::Config config {
            .rclcpp       = rclcpp,                     // 当前组件持有的 RclcppNode
            .topic        = "odom_to_camera_transform", // 发布的 topic 名
            .parent_frame = "odom_imu_link",            // 父坐标系
            .child_frame  = "camera_link",              // 子坐标系
        };
        visual_odom_to_camera = std::make_unique<visual::Transform>(config);

        action_throttler.register_action("commit_control_state_failed");
    }

    auto update() -> void override {
        using namespace rmcs_description;

        if (!rmcs_tf.ready()) [[unlikely]]
            return;

        {
            update_control_state();
            auto success = feishu.commit(control_state);

            if (!success) {
                action_throttler.dispatch("commit_control_state_failed",
                    [&] { rclcpp.info("commit control state failed!"); });
            } else {
                action_throttler.reset("commit_control_state_failed");
            }
        }

        if (feishu.updated()) {
            auto_aim_state = feishu.fetch();
            update_target_direction();
        }
    }

private:
    InputInterface<rmcs_description::Tf> rmcs_tf;

    InputInterface<double> current_gimbal_yaw;
    InputInterface<double> current_gimbal_pitch;

    RclcppNode rclcpp;
    std::unique_ptr<visual::Transform> visual_odom_to_camera;

    Feishu<RuntimeRole::Control> feishu;
    ControlState control_state;
    AutoAimState auto_aim_state;

    OutputInterface<Eigen::Vector3d> target_direction;

    FramerateCounter framerate;
    ActionThrottler action_throttler { std::chrono::seconds(1), 3 };

private:
    auto update_control_state() -> void {
        control_state.timestamp = Clock::now();

        auto odom_to_camera_transform =
            fast_tf::lookup_transform<rmcs_description::OdomImu, rmcs_description::CameraLink>(
                *rmcs_tf);

        control_state.odom_to_camera_transform.position = odom_to_camera_transform.translation();
        control_state.odom_to_camera_transform.orientation =
            Eigen::Quaterniond(odom_to_camera_transform.rotation());

        visual_odom_to_camera->move(control_state.odom_to_camera_transform.position,
            control_state.odom_to_camera_transform.orientation);
        visual_odom_to_camera->update();

        // TODO:无敌状态下的装甲板需要从裁判系统获取并在此更新
        control_state.invincible_devices = DeviceIds::None();

        // TODO:弹速需要进一步确认
        control_state.bullet_speed = 20;
        control_state.yaw          = *current_gimbal_yaw;
        control_state.pitch        = *current_gimbal_pitch;
    }

    auto update_target_direction() -> void {
        auto yaw   = auto_aim_state.yaw;
        auto pitch = auto_aim_state.pitch;

        auto direction = Eigen::Vector3d {};
        direction.x()  = std::cos(pitch) * std::cos(yaw);
        direction.y()  = std::cos(pitch) * std::sin(yaw);
        direction.z()  = std::sin(pitch);

        *target_direction = direction;
    }
};

} // namespace rmcs

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs::AutoAimComponent, rmcs_executor::Component)
