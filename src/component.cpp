#include "kernel/feishu.hpp"
#include "module/debug/framerate.hpp"
#include "utility/rclcpp/node.hpp"
#include "utility/rclcpp/visual/transform.hpp"
#include "utility/shared/context.hpp"

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

        using namespace std::chrono_literals;
        framerate.set_interval(2s);

        visual::Transform::Config config {
            .rclcpp       = rclcpp,                     // 当前组件持有的 RclcppNode
            .topic        = "odom_to_camera_transform", // 发布的 topic 名
            .parent_frame = "odom_imu_link",            // 父坐标系
            .child_frame  = "camera_link",              // 子坐标系
        };
        visual_odom_to_camera = std::make_unique<visual::Transform>(config);
    }

    auto update() -> void override {
        using namespace rmcs_description;

        if (!rmcs_tf.ready()) [[unlikely]]
            return;

        {
            control_state.timestamp = Clock::now();

            auto odom_to_camera_transform =
                fast_tf::lookup_transform<rmcs_description::OdomImu, rmcs_description::CameraLink>(
                    *rmcs_tf);

            control_state.odom_to_camera_transform.position =
                odom_to_camera_transform.translation();
            control_state.odom_to_camera_transform.orientation =
                Eigen::Quaterniond(odom_to_camera_transform.rotation());

            visual_odom_to_camera->move(control_state.odom_to_camera_transform.position,
                control_state.odom_to_camera_transform.orientation);
            visual_odom_to_camera->update();

            // TODO:无敌状态下的装甲板需要从裁判系统获取并在此更新
            control_state.invincible_devices = DeviceIds::None();

            // TODO:弹速需要进一步确认
            control_state.bullet_speed = 25;
            auto success               = feishu.commit(control_state);

            // TODO:添加错误输出的时间间隔和次数限制
            if (!success) rclcpp.info("commit control state failed!");
        }

        if (feishu.updated()) {
            auto_aim_state = feishu.fetch();
        }
    }

private:
    InputInterface<rmcs_description::Tf> rmcs_tf;

    RclcppNode rclcpp;
    std::unique_ptr<visual::Transform> visual_odom_to_camera;

    Feishu<RuntimeRole::Control> feishu;
    ControlState control_state;
    AutoAimState auto_aim_state;

    FramerateCounter framerate;

private:
};

} // namespace rmcs

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs::AutoAimComponent, rmcs_executor::Component)
