#include "kernel/feishu.hpp"
#include "module/debug/action_throttler.hpp"
#include "module/debug/framerate.hpp"
#include "utility/rclcpp/node.hpp"
#include "utility/rclcpp/visual/transform.hpp"
#include "utility/shared/context.hpp"

#include <chrono>
#include <cmath>
#include <limits>
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

        register_output("/gimbal/auto_aim/auto_aim_enabled", gimbal_takeover, false);
        register_output(
            "/gimbal/auto_aim/control_direction", target_direction, Eigen::Vector3d::Zero());
        register_output("/gimbal/auto_aim/shoot_enable", shoot_permitted, false);

        using namespace std::chrono_literals;
        framerate.set_interval(2s);

        visual::Transform::Config config {
            .rclcpp       = rclcpp,                     // 当前组件持有的 RclcppNode
            .topic        = "odom_to_camera_transform", // 发布的 topic 名
            .parent_frame = "odom_imu_link",            // 父坐标系
            .child_frame  = "camera_link",              // 子坐标系
        };
        visual_odom_to_camera = std::make_unique<visual::Transform>(config);

        action_throttler.register_action("tf_not_ready");
        action_throttler.register_action("commit_control_state_failed");
    }

    auto update() -> void override {
        using namespace rmcs_description;

        if (!rmcs_tf.ready()) [[unlikely]] {
            action_throttler.dispatch("tf_not_ready", [&] { rclcpp.warn("rmcs_tf is not ready"); });
            control_state.set_identity();
            reset_control_commands();
            return;
        }
        {
            update_gimbal_direction();
            update_control_state();

            auto success = feishu.commit(control_state);
            if (!success) {
                action_throttler.dispatch("commit_control_state_failed",
                    [&] { rclcpp.info("commit control state failed!"); });
            } else {
                action_throttler.reset("commit_control_state_failed");
            }
        }
        {
            refresh_auto_aim_state();
            apply_auto_aim_state(current_auto_aim_state());
        }
    }

private:
    static constexpr auto auto_aim_state_timeout_ { std::chrono::milliseconds { 100 } };

    InputInterface<rmcs_description::Tf> rmcs_tf;

    double current_gimbal_yaw { std::numeric_limits<double>::quiet_NaN() };
    double current_gimbal_pitch { std::numeric_limits<double>::quiet_NaN() };

    RclcppNode rclcpp;
    std::unique_ptr<visual::Transform> visual_odom_to_camera;

    Feishu<RuntimeRole::Control> feishu;
    ControlState control_state;
    AutoAimState auto_aim_state;
    bool auto_aim_state_received_ { false };

    OutputInterface<bool> gimbal_takeover;
    OutputInterface<bool> shoot_permitted;
    OutputInterface<Eigen::Vector3d> target_direction;

    FramerateCounter framerate;
    ActionThrottler action_throttler { std::chrono::seconds(1), 233 };

    auto refresh_auto_aim_state() -> void {
        if (!feishu.updated()) return;

        auto_aim_state           = feishu.fetch();
        auto_aim_state_received_ = true;
    }

    auto has_valid_auto_aim_state() const -> bool {
        return auto_aim_state_received_
            && Clock::now() - auto_aim_state.timestamp <= auto_aim_state_timeout_;
    }

    auto current_auto_aim_state() const -> AutoAimState {
        auto state = auto_aim_state;
        if (has_valid_auto_aim_state()) {
            return state;
        }

        state.gimbal_takeover = false;
        state.shoot_permitted = false;
        state.yaw             = current_gimbal_yaw;
        state.pitch           = current_gimbal_pitch;
        return state;
    }

    auto apply_auto_aim_state(const AutoAimState& state) -> void {
        *gimbal_takeover = state.gimbal_takeover;
        *shoot_permitted = state.shoot_permitted;
        update_target_direction(state);
    }

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

        control_state.yaw   = current_gimbal_yaw;
        control_state.pitch = current_gimbal_pitch;
    }

    auto update_target_direction(const AutoAimState& state) -> void {
        const auto& [yaw, pitch] = std::tie(state.yaw, state.pitch);

        // clang-format off
        *target_direction = Eigen::Vector3d {
            std::cos(pitch) * std::cos(yaw),
            std::cos(pitch) * std::sin(yaw), 
            std::sin(pitch)
        };
        // clang-format on
    }

    auto reset_control_commands() -> void {
        *gimbal_takeover  = false;
        *shoot_permitted  = false;
        *target_direction = Eigen::Vector3d::Zero();
    }

    auto update_gimbal_direction() -> void {
        auto odom_to_muzzle_transform =
            fast_tf::lookup_transform<rmcs_description::OdomImu, rmcs_description::PitchLink>(
                *rmcs_tf);

        auto quat = Eigen::Quaterniond { odom_to_muzzle_transform.toRotationMatrix() };

        auto current_muzzle_direction = quat * Eigen::Vector3d::UnitX();

        current_gimbal_yaw = std::atan2(current_muzzle_direction.y(), current_muzzle_direction.x());
        current_gimbal_pitch = std::atan2(current_muzzle_direction.z(),
            std::hypot(current_muzzle_direction.x(), current_muzzle_direction.y()));
    }
};

} // namespace rmcs

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs::AutoAimComponent, rmcs_executor::Component)
