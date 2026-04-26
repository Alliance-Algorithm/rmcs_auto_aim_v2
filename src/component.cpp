#include "adapter/sentry.hpp"
#include "kernel/feishu.hpp"
#include "module/debug/action_throttler.hpp"
#include "module/debug/framerate.hpp"
#include "utility/rclcpp/node.hpp"
#include "utility/rclcpp/visual/transform.hpp"
#include "utility/shared/context.hpp"

#include <chrono>
#include <cmath>
#include <limits>

#include <rmcs_executor/component.hpp>

namespace rmcs {

using namespace util;
using namespace kernel;

class AutoAimComponent final : public rmcs_executor::Component {
public:
    explicit AutoAimComponent() noexcept
        : adapter { *this }
        , rclcpp { get_component_name() } {

        register_output("/gimbal/auto_aim/auto_aim_enabled", gimbal_takeover, false);
        register_output(
            "/gimbal/auto_aim/control_direction", target_direction, Eigen::Vector3d::Zero());
        register_output("/gimbal/auto_aim/shoot_enable", shoot_permitted, false);

        using namespace std::chrono_literals;
        framerate.set_interval(2s);

        const auto config = visual::Transform::Config {
            .rclcpp       = rclcpp,
            .topic        = "odom_to_camera_transform",
            .parent_frame = Adapter::kParentFrame,
            .child_frame  = "camera_link",
        };
        visual_odom_to_camera = std::make_unique<visual::Transform>(config);

        action_throttler.register_action("tf_not_ready");
        action_throttler.register_action("commit_control_state_failed");
    }

    auto update() -> void override {
        if (!adapter.ready()) [[unlikely]] {
            action_throttler.dispatch("tf_not_ready", [&] { rclcpp.warn("adapter is not ready"); });
            command = ControlState::kInvalid();

            const auto state  = AutoAimState::kInvalid();
            *gimbal_takeover  = state.gimbal_takeover;
            *shoot_permitted  = state.shoot_permitted;
            *target_direction = compute_target_direction(state);
            return;
        }

        update_control_state();
        feishu.send(command);
        action_throttler.reset("commit_control_state_failed");

        if (feishu.heartbeat()) {
            if (auto latest = feishu.latest()) {
                context = *latest;
            }
            auto_aim_state_received_ = true;
        }

        const auto state =
            auto_aim_state_received_ && Clock::now() - context.timestamp <= kAutoAimTimeout
            ? context
            : AutoAimState::kInvalid();

        *gimbal_takeover  = state.gimbal_takeover;
        *shoot_permitted  = state.shoot_permitted;
        *target_direction = compute_target_direction(state);
    }

private:
    static constexpr auto kAutoAimTimeout = std::chrono::milliseconds { 100 };

    Adapter adapter;

    double current_gimbal_yaw { std::numeric_limits<double>::quiet_NaN() };
    double current_gimbal_pitch { std::numeric_limits<double>::quiet_NaN() };

    RclcppNode rclcpp;
    std::unique_ptr<visual::Transform> visual_odom_to_camera;

    Feishu<ControlState, AutoAimState> feishu;
    ControlState command;
    AutoAimState context;
    bool auto_aim_state_received_ { false };

    OutputInterface<bool> gimbal_takeover;
    OutputInterface<bool> shoot_permitted;
    OutputInterface<Eigen::Vector3d> target_direction;

    FramerateCounter framerate;
    ActionThrottler action_throttler { std::chrono::seconds(1), 233 };

    static auto compute_target_direction(const AutoAimState& state) -> Eigen::Vector3d {
        if (!state.gimbal_takeover || !std::isfinite(state.yaw) || !std::isfinite(state.pitch)) {
            return Eigen::Vector3d::Zero();
        }

        const auto& [yaw, pitch] = std::tie(state.yaw, state.pitch);

        return {
            std::cos(pitch) * std::cos(yaw),
            std::cos(pitch) * std::sin(yaw),
            std::sin(pitch),
        };
    }

    std::uint8_t publish_count = 0;
    auto update_control_state() -> void {
        command.timestamp = Clock::now();

        auto dir             = adapter.barrel_direction();
        current_gimbal_yaw   = std::atan2(dir.y(), dir.x());
        current_gimbal_pitch = std::atan2(-dir.z(), std::hypot(dir.x(), dir.y()));

        auto iso                                     = adapter.camera_transform();
        command.odom_to_camera_transform.position    = iso.translation();
        command.odom_to_camera_transform.orientation = Eigen::Quaterniond(iso.rotation());

        visual_odom_to_camera->move(command.odom_to_camera_transform.position,
            command.odom_to_camera_transform.orientation);

        if (publish_count++ > 100) {
            publish_count = 0;
            visual_odom_to_camera->update();
        }

        // TODO:无敌状态下的装甲板需要从裁判系统获取并在此更新
        command.invincible_devices = DeviceIds::None();

        command.yaw   = current_gimbal_yaw;
        command.pitch = current_gimbal_pitch;
    }
};

} // namespace rmcs

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs::AutoAimComponent, rmcs_executor::Component)
