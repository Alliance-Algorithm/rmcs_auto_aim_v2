#include "kernel/feishu.hpp"
#include "module/debug/action_throttler.hpp"
#include "module/debug/framerate.hpp"
#include "utility/clock.hpp"
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
using Clock = util::Clock;

class AutoAimComponent final : public rmcs_executor::Component {
public:
    explicit AutoAimComponent() noexcept
        : rclcpp { get_component_name() } {

        register_input("/predefined/timestamp", predefined_timestamp_);
        register_input("/tf", rmcs_tf);
        register_input("/camera/trigger/seq", camera_trigger_seq_);
        register_input("/camera/trigger/timestamp", camera_trigger_timestamp_);

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
        action_throttler.register_action("commit_camera_trigger_failed");
        action_throttler.register_action("camera_trigger_gap_detected");
    }

    auto update() -> void override {
        if (!rmcs_tf.ready()) [[unlikely]] {
            handle_tf_not_ready();
            return;
        }

        publish_control_state();
        forward_auto_aim_outputs();
    }

private:
    static constexpr auto auto_aim_state_timeout { std::chrono::milliseconds { 100 } };

    InputInterface<Clock::time_point> predefined_timestamp_;
    InputInterface<rmcs_description::Tf> rmcs_tf;

    InputInterface<std::uint64_t> camera_trigger_seq_;
    InputInterface<Clock::time_point> camera_trigger_timestamp_;

    double current_gimbal_yaw { std::numeric_limits<double>::quiet_NaN() };
    double current_gimbal_pitch { std::numeric_limits<double>::quiet_NaN() };

    RclcppNode rclcpp;
    std::unique_ptr<visual::Transform> visual_odom_to_camera;

    Feishu<RuntimeRole::Control> feishu;
    Channel<CameraTriggerEvent> camera_trigger_channel;
    ControlState control_state;
    AutoAimState auto_aim_state;
    bool auto_aim_state_received_ { false };
    std::uint64_t last_committed_camera_trigger_seq_ { 0 };

    OutputInterface<bool> gimbal_takeover;
    OutputInterface<bool> shoot_permitted;
    OutputInterface<Eigen::Vector3d> target_direction;

    FramerateCounter framerate;
    ActionThrottler action_throttler { std::chrono::seconds(1), 233 };

    auto has_fresh_auto_aim_state() const -> bool {
        return auto_aim_state_received_
            && Clock::now() - auto_aim_state.timestamp <= auto_aim_state_timeout;
    }

    static auto make_invalid_auto_aim_state() -> AutoAimState {
        auto state = AutoAimState {};
        state.reset();
        return state;
    }

    auto resolve_auto_aim_state() -> AutoAimState {
        if (feishu.updated()) {
            auto_aim_state           = feishu.fetch();
            auto_aim_state_received_ = true;
        }

        if (has_fresh_auto_aim_state()) {
            return auto_aim_state;
        }

        return make_invalid_auto_aim_state();
    }

    auto publish_auto_aim_outputs(const AutoAimState& state) -> void {
        *gimbal_takeover  = state.gimbal_takeover;
        *shoot_permitted  = state.shoot_permitted;
        *target_direction = compute_target_direction(state);
    }

    static auto compute_target_direction(const AutoAimState& state) -> Eigen::Vector3d {
        if (!state.has_control_direction()) {
            return Eigen::Vector3d::Zero();
        }

        const auto& [yaw, pitch] = std::tie(state.yaw, state.pitch);

        // clang-format off
        return Eigen::Vector3d {
            std::cos(pitch) * std::cos(yaw),
            std::cos(pitch) * std::sin(yaw), 
            std::sin(pitch)
        };
        // clang-format on
    }

    auto forward_auto_aim_outputs() -> void { publish_auto_aim_outputs(resolve_auto_aim_state()); }

    auto handle_tf_not_ready() -> void {
        action_throttler.dispatch("tf_not_ready", [&] { rclcpp.warn("rmcs_tf is not ready"); });
        control_state.reset();
        publish_auto_aim_outputs(make_invalid_auto_aim_state());
    }

    auto publish_control_state() -> void {
        update_gimbal_direction();
        update_control_state();
        publish_camera_trigger_event();

        auto success = feishu.commit(control_state);
        if (!success) {
            action_throttler.dispatch("commit_control_state_failed",
                [&] { rclcpp.info("commit control state failed!"); });
        } else {
            action_throttler.reset("commit_control_state_failed");
        }
    }

    auto publish_camera_trigger_event() -> void {
        auto trigger_seq = *camera_trigger_seq_;
        if (trigger_seq == 0 || trigger_seq == last_committed_camera_trigger_seq_) {
            return;
        }

        if (last_committed_camera_trigger_seq_ != 0
            && trigger_seq > last_committed_camera_trigger_seq_ + 1) {
            action_throttler.dispatch("camera_trigger_gap_detected", [&] {
                rclcpp.warn("Camera trigger gap detected: last={}, current={}",
                    last_committed_camera_trigger_seq_, trigger_seq);
            });
        } else {
            action_throttler.reset("camera_trigger_gap_detected");
        }

        auto success = camera_trigger_channel.commit(CameraTriggerEvent {
            .seq       = trigger_seq,
            .timestamp = *camera_trigger_timestamp_,
        });
        if (!success) {
            action_throttler.dispatch("commit_camera_trigger_failed",
                [&] { rclcpp.info("commit camera trigger event failed!"); });
            return;
        }

        last_committed_camera_trigger_seq_ = trigger_seq;
        action_throttler.reset("commit_camera_trigger_failed");
    }

    auto update_control_state() -> void {
        control_state.timestamp = *predefined_timestamp_;

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

    auto update_gimbal_direction() -> void {
        using namespace rmcs_description;

        auto odom_to_pitch_transform =
            fast_tf::lookup_transform<rmcs_description::OdomImu, rmcs_description::PitchLink>(
                *rmcs_tf);

        auto quat = Eigen::Quaterniond { odom_to_pitch_transform.toRotationMatrix() };

        auto current_pitch_direction = quat * Eigen::Vector3d::UnitX();

        current_gimbal_yaw   = std::atan2(current_pitch_direction.y(), current_pitch_direction.x());
        current_gimbal_pitch = std::atan2(current_pitch_direction.z(),
            std::hypot(current_pitch_direction.x(), current_pitch_direction.y()));
    }
};

} // namespace rmcs

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs::AutoAimComponent, rmcs_executor::Component)
