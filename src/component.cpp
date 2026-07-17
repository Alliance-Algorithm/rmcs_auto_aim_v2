#include "kernel/auto_aim.hpp"
#include "kernel/fire_control.hpp"
#include "utility/rclcpp/node.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
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
using namespace rmcs::kernel;

class AutoAimComponent final : public rmcs_executor::Component {
    static inline const auto kTNaN = Eigen::Vector3d { kNaN, kNaN, kNaN };

private:
    AutoAim auto_aim { };
    RclcppNode rclcpp { get_component_name() };

    std::unique_ptr<FireController> fire;

    bool manual_shoot = false;

    struct GimbalState {
        Timestamp timestamp;

        Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
        Eigen::Vector3d gyro_body      = Eigen::Vector3d::Zero();
    } gimbal;
    std::mutex gimbal_mutex;

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
                {
                    std::lock_guard lock { gimbal_mutex };
                    gimbal.orientation = frame->imu_snapshot;
                    gimbal.gyro_body   = frame->gyro_body;
                    gimbal.timestamp   = frame->exposure_timestamp;
                }
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

    InputInterface<rmcs_msgs::RobotId> robot_id;
    InputInterface<rmcs_msgs::Switch> rswitch;
    InputInterface<rmcs_msgs::Switch> lswitch;
    InputInterface<rmcs_msgs::Mouse> mouse;

    OutputInterface<bool> should_track;
    OutputInterface<bool> should_shoot;
    OutputInterface<bool> single_shoot;
    OutputInterface<Eigen::Vector3d> track_target;
    OutputInterface<Eigen::Vector3d> robot_center;

    double max_yaw_acc = 100.;
    double max_yaw_vel = 3.;

    Timestamp last_yaw_vel_timestamp;
    double last_yaw_velocity = kNaN;

    Trackable::Unique current_trackable { };

public:
    AutoAimComponent() {
        register_input("/gimbal/auto_aim/camera_frame", camera_frame, false);
        register_input("/referee/id", robot_id, false);
        register_input("/remote/switch/right", rswitch, false);
        register_input("/remote/switch/left", lswitch, false);
        register_input("/remote/mouse", mouse, false);

        register_output("/auto_aim/should_control", should_track, false);
        register_output("/auto_aim/should_shoot", should_shoot, false);
        register_output("/auto_aim/single_shoot", single_shoot, false);
        register_output("/auto_aim/control_direction", track_target, kTNaN);
        register_output("/auto_aim/robot_center", robot_center, kTNaN);

        const auto& params = rclcpp.params();

        manual_shoot = params.get_bool("manual_shoot");

        if (auto config = util::serialize<FireController::Config>("fire_control", params)) {
            fire = std::make_unique<FireController>(*config);
        } else {
            throw std::runtime_error { config.error() };
        }

        const auto t = params.get<std::vector<double>>("camera_translation");
        if (t.size() == 3) {
            auto_aim.with_context([&](AutoAim::Context& ctx) {
                ctx.camera_translation = Translation { t[0], t[1], t[2] };
            });
        } else {
            rclcpp.error("Parameter 'camera_translation' expects 3 elements, got {}", t.size());
        }
    }

    ~AutoAimComponent() override {
        camera_frame_stop_requested.store(true, std::memory_order::relaxed);
        camera_frame_worker.request_stop();
        camera_frame_event_count.fetch_add(1, std::memory_order::release);
        camera_frame_event_count.notify_one();
    }

    auto before_updating() -> void override {
        using namespace rmcs_msgs;
        if (!robot_id.ready()) {
            robot_id.make_and_bind_directly(RobotId::UNKNOWN);
        }
    }

    auto update() -> void override {
        auto gimbal_q    = Eigen::Quaterniond { };
        auto gimbal_gyro = Eigen::Vector3d { };
        auto gimbal_time = Timestamp { };
        {
            std::lock_guard lock { gimbal_mutex };
            gimbal_q    = gimbal.orientation;
            gimbal_gyro = gimbal.gyro_body;
            gimbal_time = gimbal.timestamp;
        }

        if (gimbal_time != Timestamp { } && gimbal_time != last_yaw_vel_timestamp) {
            const auto velocity = (gimbal_q * gimbal_gyro).z();
            if (std::isfinite(velocity)) {
                max_yaw_vel = std::max(max_yaw_vel, std::abs(velocity));
                if (std::isfinite(last_yaw_velocity)) {
                    const auto dt =
                        std::chrono::duration<double>(gimbal_time - last_yaw_vel_timestamp).count();
                    if (dt > 1e-6) {
                        max_yaw_acc =
                            std::max(max_yaw_acc, std::abs((velocity - last_yaw_velocity) / dt));
                    }
                }
                last_yaw_vel_timestamp = gimbal_time;
                last_yaw_velocity      = velocity;
            }
        }

        using namespace rmcs_msgs;
        const auto track_intent =
            (mouse.ready() && mouse->right) || (rswitch.ready() && *rswitch == Switch::UP);
        const auto shoot_intent =
            (mouse.ready() && mouse->left) || (lswitch.ready() && *lswitch == Switch::DOWN);

        auto_aim.with_context([=, this](AutoAim::Context& ctx) {
            ctx.track_intent = track_intent;

            ctx.max_yaw_vel = std::max(max_yaw_vel, ctx.max_yaw_vel);
            ctx.max_yaw_acc = std::max(max_yaw_acc, ctx.max_yaw_acc);

            ctx.id = *robot_id;

            /// TODO:
            /// 跟踪目标，用于适配后期可能存在的需求，
            /// 比如无人机前哨 Only 模式，哨兵滤除特殊兵种等
            ctx.track_ids = DeviceIds::Full();
        });

        if (auto_aim.command_updated()) {
            auto_aim.with_command([this](const AutoAim::Command& cmd) {
                if (cmd.trackable) {
                    current_trackable = cmd.trackable->clone();
                }
            });
        }

        const auto now = Clock::now();
        using namespace std::chrono_literals;

        if (current_trackable && now - current_trackable->get_timestamp() > 100ms) {
            current_trackable.reset();

            *should_track = false;
            *should_shoot = false;
            *single_shoot = false;
            *track_target = kTNaN;
            *robot_center = kTNaN;

            auto_aim.with_context([](AutoAim::Context& ctx) { ctx.addition = { }; });
        }

        if (current_trackable) {
            *single_shoot = current_trackable->id() == DeviceId::RUNE;

            const auto dir = gimbal_q * Eigen::Vector3d::UnitX();
            fire->update({
                .timestamp   = now,
                .yaw         = std::atan2(+dir.y(), dir.x()),
                .pitch       = std::atan2(-dir.z(), std::hypot(dir.x(), dir.y())),
                .max_yaw_vel = max_yaw_vel,
                .max_yaw_acc = max_yaw_acc,
            });

            if (auto aimed = fire->aim(*current_trackable)) {
                *should_track = true;
                *should_shoot = manual_shoot ? (aimed->shoot && shoot_intent) : aimed->shoot;

                *robot_center = aimed->center.make<Eigen::Vector3d>();
                *track_target = aimed->target.make<Eigen::Vector3d>();

                auto_aim.with_context([&](AutoAim::Context& ctx) {
                    auto& addition = ctx.addition;

                    addition.attack       = aimed->attack;
                    addition.aim_yaw      = aimed->aim_yaw;
                    addition.raw_yaw      = aimed->raw_yaw;
                    addition.pitch        = aimed->pitch;
                    addition.pre_aim      = aimed->pre_aim;
                    addition.should_track = true;
                    addition.should_shoot = aimed->shoot;
                });
            }
        }
    }
};

} // namespace rmcs

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs::AutoAimComponent, rmcs_executor::Component)
