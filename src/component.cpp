#include "adapter/sentry.hpp"
#include "kernel/feishu.hpp"
#include "module/debug/action_throttler.hpp"
#include "module/debug/framerate.hpp"
#include "utility/rclcpp/node.hpp"
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

        register_output("/auto_aim/should_control", should_control, false);
        register_output("/auto_aim/control_direction", target_direction, Eigen::Vector3d::Zero());
        register_output("/auto_aim/should_shoot", should_shoot, false);

        using namespace std::chrono_literals;
        framerate.set_interval(2s);

        action_throttler.register_action("adapter");
        action_throttler.register_action("feishu");
    }

    auto update() -> void override {
        if (!adapter.ready()) [[unlikely]] {
            action_throttler.dispatch("adapter", [&] { rclcpp.warn("adapter is not ready"); });

            *should_control = false;
            *should_shoot   = false;

            feishu.send(SystemContext::kInvalid());
            return;
        }
        action_throttler.reset("adapter");

        feishu.send(make_context());
        action_throttler.reset("feishu");

        if (!feishu.heartbeat()) return;

        auto command = *feishu.latest();
        if (Clock::now() - command.timestamp > kAutoAimTimeout) {
            *should_control = false;
            *should_shoot   = false;
        }

        *should_control = command.should_control;
        *should_shoot   = command.should_shoot;

        if (!*should_control) return;

        const auto pitch  = command.pitch;
        const auto yaw    = command.yaw;
        *target_direction = Eigen::Vector3d {
            std::cos(pitch) * std::cos(yaw),
            std::cos(pitch) * std::sin(yaw),
            std::sin(pitch),
        };
    }

private:
    static constexpr auto kAutoAimTimeout = std::chrono::milliseconds { 100 };

    Adapter adapter;

    double current_gimbal_yaw { std::numeric_limits<double>::quiet_NaN() };
    double current_gimbal_pitch { std::numeric_limits<double>::quiet_NaN() };

    RclcppNode rclcpp;

    Feishu<SystemContext, AutoAimState> feishu;

    OutputInterface<bool> should_control;
    OutputInterface<bool> should_shoot;
    OutputInterface<Eigen::Vector3d> target_direction;

    FramerateCounter framerate;
    ActionThrottler action_throttler { std::chrono::seconds(1), 233 };

    auto make_context() -> SystemContext {
        auto context = SystemContext { };

        context.timestamp = Clock::now();

        auto dir             = adapter.barrel_direction();
        current_gimbal_yaw   = std::atan2(dir.y(), dir.x());
        current_gimbal_pitch = std::atan2(-dir.z(), std::hypot(dir.x(), dir.y()));

        auto iso                             = adapter.camera_transform();
        context.camera_transform.translation = iso.translation();
        context.camera_transform.orientation = Eigen::Quaterniond(iso.rotation());

        // TODO:无敌状态下的装甲板需要从裁判系统获取并在此更新
        context.invincible_devices = DeviceIds::None();

        context.yaw   = current_gimbal_yaw;
        context.pitch = current_gimbal_pitch;

        return context;
    }
};

} // namespace rmcs

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs::AutoAimComponent, rmcs_executor::Component)
