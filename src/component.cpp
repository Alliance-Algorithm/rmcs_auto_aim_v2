#include "kernel/auto_aim.hpp"
#include "utility/rclcpp/node.hpp"

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <memory>
#include <thread>

#include <eigen3/Eigen/Geometry>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/camera_frame.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/robot_id.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs {

using namespace rmcs::util;

class AutoAimComponent final : public rmcs_executor::Component {
    static inline const auto kTNaN = Eigen::Vector3d { kNaN, kNaN, kNaN };

private:
    AutoAim auto_aim { };
    RclcppNode rclcpp { get_component_name() };

    bool manual_shoot = false;

    EventInputInterface<std::shared_ptr<const rmcs_msgs::CameraFrame>> camera_frame {
        [this](const std::shared_ptr<const rmcs_msgs::CameraFrame>& frame) {
            if (!frame || camera_frame_stop_requested.load(std::memory_order::relaxed)) return;

            latest_camera_frame.store(frame, std::memory_order::release);
            camera_frame_event_count.fetch_add(1, std::memory_order::release);
            camera_frame_event_count.notify_one();
        },
    };

    std::atomic<std::shared_ptr<const rmcs_msgs::CameraFrame>> latest_camera_frame { nullptr };
    std::atomic<std::uint32_t> camera_frame_event_count { 0 };
    std::atomic<bool> camera_frame_stop_requested { false };
    std::jthread camera_frame_worker { [this](const std::stop_token& stop) {
        while (!stop.stop_requested()
            && !camera_frame_stop_requested.load(std::memory_order::relaxed)) {
            if (auto frame = latest_camera_frame.exchange(nullptr, std::memory_order::acq_rel)) {
                auto_aim.process(Image { std::move(frame) });
                continue;
            }

            const auto old = camera_frame_event_count.load(std::memory_order::relaxed);
            if (!latest_camera_frame.load(std::memory_order::acquire) && !stop.stop_requested()
                && !camera_frame_stop_requested.load(std::memory_order::relaxed)) {
                camera_frame_event_count.wait(old, std::memory_order::acquire);
            }
        }
    } };

    InputInterface<Eigen::Isometry3d> camera_transform;
    InputInterface<Eigen::Vector3d> barrel_direction;
    InputInterface<double> yaw_velocity;
    InputInterface<rmcs_msgs::RobotId> robot_id;
    InputInterface<rmcs_msgs::Switch> rswitch;
    InputInterface<rmcs_msgs::Switch> lswitch;
    InputInterface<rmcs_msgs::Mouse> mouse;

    OutputInterface<bool> should_track;
    OutputInterface<bool> should_shoot;
    OutputInterface<bool> single_shoot;
    OutputInterface<Eigen::Vector3d> target_direction;
    OutputInterface<Eigen::Vector3d> robot_center;

    Timestamp last_yaw_vel_timestamp;
    double last_yaw_velocity = kNaN;

    Timestamp last_command_timestamp;

public:
    AutoAimComponent() noexcept {
        register_input("/gimbal/auto_aim/camera_frame", camera_frame, false);
        register_input("/auto_aim/camera_transform", camera_transform, false);
        register_input("/auto_aim/barrel_direction", barrel_direction, false);
        register_input("/auto_aim/yaw_velocity", yaw_velocity, false);
        register_input("/referee/id", robot_id, false);
        register_input("/remote/switch/right", rswitch, false);
        register_input("/remote/switch/left", lswitch, false);
        register_input("/remote/mouse", mouse, false);

        register_output("/auto_aim/should_control", should_track, false);
        register_output("/auto_aim/should_shoot", should_shoot, false);
        register_output("/auto_aim/single_shoot", single_shoot, false);
        register_output("/auto_aim/control_direction", target_direction, kTNaN);
        register_output("/auto_aim/robot_center", robot_center, kTNaN);

        const auto& params = rclcpp.params();

        manual_shoot = params.get_bool("manual_shoot");
    }

    ~AutoAimComponent() override {
        camera_frame_stop_requested.store(true, std::memory_order::relaxed);
        camera_frame_worker.request_stop();
        camera_frame_event_count.fetch_add(1, std::memory_order::release);
        camera_frame_event_count.notify_one();
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
        auto max_yaw_vel = double { 0.0 };
        auto max_yaw_acc = double { 0.0 };
        if (yaw_velocity.ready() && std::isfinite(*yaw_velocity)) {
            const auto now      = Clock::now();
            const auto velocity = *yaw_velocity;

            max_yaw_vel = std::max(max_yaw_vel, std::abs(velocity));
            if (std::isfinite(last_yaw_velocity)) {
                const auto dt = std::chrono::duration<double>(now - last_yaw_vel_timestamp).count();
                if (dt > 1e-6) {
                    max_yaw_acc =
                        std::max(max_yaw_acc, std::abs((velocity - last_yaw_velocity) / dt));
                }
            }
            last_yaw_vel_timestamp = now;
            last_yaw_velocity      = velocity;
        }

        auto_aim.with_context([=, this](AutoAim::Context& ctx) {
            auto frame      = AutoAim::Context::TransformFrame { };
            frame.timestamp = Clock::now();

            const auto& dir = *barrel_direction;
            frame.yaw       = std::atan2(dir.y(), dir.x());
            frame.pitch     = std::atan2(-dir.z(), std::hypot(dir.x(), dir.y()));

            const auto& iso             = *camera_transform;
            frame.transform.translation = iso.translation();
            frame.transform.orientation = Eigen::Quaterniond { iso.rotation() };

            ctx.transforms.push_back(frame);
            if (ctx.transforms.size() > 100) {
                ctx.transforms.pop_front();
            }

            using namespace rmcs_msgs;
            ctx.track_intent =
                (mouse.ready() && mouse->right) || (rswitch.ready() && *rswitch == Switch::UP);

            ctx.max_yaw_vel = std::max(max_yaw_vel, ctx.max_yaw_vel);
            ctx.max_yaw_acc = std::max(max_yaw_acc, ctx.max_yaw_acc);

            ctx.id = *robot_id;

            /// TODO:
            /// 跟踪目标，用于适配后期可能存在的需求，
            /// 比如无人机前哨 Only 模式，哨兵滤除特殊兵种等
            ctx.track_ids = DeviceIds::Full();
        });

        const auto now = std::chrono::steady_clock::now();
        if (auto_aim.command_updated()) {
            auto_aim.with_command([=, this](const AutoAim::Command& cmd) {
                using namespace std::chrono_literals;
                if (Clock::now() - cmd.timestamp > 100ms) return;

                using namespace rmcs_msgs;
                const auto shoot_intent =
                    (mouse.ready() && mouse->left) || (lswitch.ready() && *lswitch == Switch::DOWN);
                *should_shoot =
                    manual_shoot ? (cmd.should_shoot && shoot_intent) : cmd.should_shoot;

                *should_track = cmd.should_track;
                *single_shoot = cmd.single_shoot;
                *robot_center = cmd.robot_center.make<Eigen::Vector3d>();

                if (!cmd.should_track) return;

                const auto pitch  = cmd.pitch;
                const auto yaw    = cmd.yaw;
                *target_direction = Eigen::Vector3d {
                    +std::cos(pitch) * std::cos(yaw),
                    +std::cos(pitch) * std::sin(yaw),
                    -std::sin(pitch),
                };
            });
            last_command_timestamp = now;
        }

        using namespace std::chrono_literals;
        if (now - last_command_timestamp > 100ms) {
            *should_track     = false;
            *should_shoot     = false;
            *target_direction = kTNaN;
        }
    }
};

} // namespace rmcs

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs::AutoAimComponent, rmcs_executor::Component)
