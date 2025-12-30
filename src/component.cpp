#include "kernel/feishu.hpp"
#include "module/debug/framerate.hpp"
#include "utility/rclcpp/node.hpp"
#include "utility/shared/context.hpp"

#include <eigen3/Eigen/Geometry>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs {

using namespace rmcs::util;
using namespace kernel;

class AutoAimComponent final : public rmcs_executor::Component {
public:
    explicit AutoAimComponent() noexcept
        : rclcpp { get_component_name() } {

        using namespace std::chrono_literals;
        framerate.set_interval(2s);
    }

    auto before_updating() -> void override { }

    auto update() -> void override {
        using namespace rmcs_description;
        if (rmcs_tf.ready()) [[likely]] {
            {
                control_state.timestamp = Clock::now();

                auto camera2odom = fast_tf::lookup_transform<rmcs_description::CameraLink,
                    rmcs_description::OdomImu>(*rmcs_tf);
                control_state.camera_to_odom_transform.posture = camera2odom.translation();
                control_state.camera_to_odom_transform.orientation =
                    Eigen::Quaterniond(camera2odom.rotation());

                // TODO:无敌状态下的装甲板需要从裁判系统获取并在此更新
                control_state.invincible_devices = DeviceIds::None();

                // TODO:弹速需要进一步确认
                control_state.bullet_speed = 25;
                feishu.commit(control_state);
            }
            {
                if (auto state = feishu.fetch<AutoAimState>()) auto_aim_state = *state;
            }
        }
    }

private:
    InputInterface<rmcs_description::Tf> rmcs_tf;

    RclcppNode rclcpp;

    Feishu<ControlSide> feishu;
    ControlState control_state;
    AutoAimState auto_aim_state;

    FramerateCounter framerate;

private:
};

} // namespace rmcs

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs::AutoAimComponent, rmcs_executor::Component)
