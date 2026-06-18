#include "kernel/auto_aim.hpp"
#include "utility/shared/context.hpp"

#include <eigen3/Eigen/Geometry>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/robot_id.hpp>

namespace rmcs {

class AutoAimComponent final : public rmcs_executor::Component {
    static inline const auto kNaN  = std::numeric_limits<double>::quiet_NaN();
    static inline const auto kTNaN = Eigen::Vector3d { kNaN, kNaN, kNaN };

private:
    AutoAim auto_aim { };

    InputInterface<Eigen::Isometry3d> camera_transform;
    InputInterface<Eigen::Vector3d> barrel_direction;
    InputInterface<rmcs_msgs::RobotId> robot_id;

    OutputInterface<bool> should_control;
    OutputInterface<bool> should_shoot;
    OutputInterface<Eigen::Vector3d> target_direction;
    OutputInterface<Eigen::Vector3d> robot_center;
    OutputInterface<double> yaw_rate;
    OutputInterface<double> pitch_rate;
    OutputInterface<double> yaw_acc;
    OutputInterface<double> pitch_acc;

    std::chrono::steady_clock::time_point last_command_timestamp;

public:
    explicit AutoAimComponent() noexcept {
        register_input("/auto_aim/camera_transform", camera_transform, false);
        register_input("/auto_aim/barrel_direction", barrel_direction, false);
        register_input("/referee/id", robot_id, false);

        register_output("/auto_aim/should_control", should_control, false);
        register_output("/auto_aim/control_direction", target_direction, kTNaN);
        register_output("/auto_aim/robot_center", robot_center, kTNaN);
        register_output("/auto_aim/should_shoot", should_shoot, false);
        register_output("/auto_aim/yaw_rate", yaw_rate, kNaN);
        register_output("/auto_aim/pitch_rate", pitch_rate, kNaN);
        register_output("/auto_aim/yaw_acc", yaw_acc, kNaN);
        register_output("/auto_aim/pitch_acc", pitch_acc, kNaN);
    }

    auto before_updating() -> void override {
        if (!camera_transform.ready()) {
            camera_transform.make_and_bind_directly(Eigen::Isometry3d::Identity());
        }
        if (!barrel_direction.ready()) {
            barrel_direction.make_and_bind_directly(Eigen::Vector3d::UnitX());
        }
        if (!robot_id.ready()) {
            robot_id.make_and_bind_directly(rmcs_msgs::RobotId::UNKNOWN);
        }
        last_command_timestamp = std::chrono::steady_clock::now();
    }

    auto update() -> void override {
        auto_aim.with_context([this](SystemContext& ctx) {
            ctx.timestamp = Clock::now();

            const auto& dir = *barrel_direction;
            ctx.yaw         = std::atan2(dir.y(), dir.x());
            ctx.pitch       = std::atan2(-dir.z(), std::hypot(dir.x(), dir.y()));

            const auto& iso = *camera_transform;
            ctx.camera_transform.translation = Translation { iso.translation() };
            ctx.camera_transform.orientation = Orientation { Eigen::Quaterniond(iso.rotation()) };

            ctx.id = *robot_id;
        });

        const auto now = std::chrono::steady_clock::now();
        if (auto_aim.command_updated()) {
            auto_aim.with_command([this](const AutoAimState& cmd) {
                using namespace std::chrono_literals;
                if (Clock::now() - cmd.timestamp > 100ms) return;

                *should_control = cmd.should_control;
                *should_shoot   = cmd.should_shoot;
                *yaw_rate       = cmd.yaw_rate;
                *pitch_rate     = cmd.pitch_rate;
                *yaw_acc        = cmd.yaw_acc;
                *pitch_acc      = cmd.pitch_acc;
                *robot_center   = {
                    cmd.robot_center.x,
                    cmd.robot_center.y,
                    cmd.robot_center.z,
                };

                if (!cmd.should_control) return;

                const auto pitch  = cmd.pitch;
                const auto yaw    = cmd.yaw;
                *target_direction = Eigen::Vector3d {
                    std::cos(pitch) * std::cos(yaw),
                    std::cos(pitch) * std::sin(yaw),
                    -std::sin(pitch),
                };
            });
            last_command_timestamp = now;
        }

        using namespace std::chrono_literals;
        if (now - last_command_timestamp > 100ms) {
            *should_control   = false;
            *should_shoot     = false;
            *target_direction = kTNaN;
        }
    }
};

} // namespace rmcs

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs::AutoAimComponent, rmcs_executor::Component)
