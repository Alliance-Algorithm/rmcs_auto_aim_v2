#include "auto_aim.hpp"

#include "kernel/capturer.hpp"
#include "kernel/fire_control.hpp"
#include "kernel/identifier.hpp"
#include "kernel/pose_estimator.hpp"
#include "kernel/tracker.hpp"
#include "kernel/visualization.hpp"

#include "utility/framerate.hpp"
#include "utility/math/linear.hpp"
#include "utility/panic.hpp"
#include "utility/rclcpp/configuration.hpp"
#include "utility/rclcpp/node.hpp"
#include "utility/rclcpp/parameters.hpp"
#include "utility/singleton/running.hpp"

#include <csignal>
#include <experimental/scope>
#include <filesystem>
#include <memory>
#include <thread>

using namespace rmcs;
using namespace rmcs::util;
using namespace rmcs::kernel;

struct AutoAim::Impl {
    AutoAim& self;
    RclcppNode node { "auto_aim" };

    Capturer cap { };
    Identifier identifier { };
    Tracker tracker { };
    PoseEstimator estimator { };
    FireControl fire { };
    Visualization visual { };

    std::unique_ptr<TrackerV2> tracker_v2;

    std::jthread worker;

    auto run(const std::stop_token& stop) -> void {
        using namespace std::chrono_literals;

        node.set_pub_topic_prefix("/rmcs/auto_aim/");

        auto framerate = FramerateCounter { };
        framerate.set_interval(std::chrono::seconds { 5 });

        while (util::get_running() && !stop.stop_requested()) {
            node.spin_once();

            auto image = cap.fetch_image();
            if (!image) continue;

            // @TODO:
            //  避免拷贝，而是把需要的具体量拿出来
            auto context = SystemContext::kIdentity();
            {
                std::lock_guard lock { self.context_mutex };
                context = self.current_context;
            }
            visual.publish(context.camera_transform, "camera_link");

            if (framerate.tick()) {
                node.info("Autoaim Framerate: {}", framerate.fps());
            }

            [[maybe_unused]] auto streamer = std::experimental::scope_exit { [&] {
                visual.draw_later( // 录制开关
                    Canvas::Text { cap.recording() ? "RECORD ON" : "RECORD OFF", { 10, 700 } });
                visual.draw_later( // 自瞄帧率
                    Canvas::Text { std::format("FPS: {}", framerate.fps()), { 10, 680 } });
                visual.update_image(*image);
            } };

            /// 1. Identify Armor
            auto armor2ds    = Armor2ds { };
            auto lightbar2ds = Lightbar2ds { };
            {
                auto result = identifier.sync_identify(*image);
                if (!result.has_value()) continue;

                for (const auto& roi : result->areas) {
                    visual.draw_later(roi);
                }
                visual.draw_later(result->armors);
                visual.draw_later(result->green_light);

                armor2ds    = std::move(result->armors);
                lightbar2ds = std::move(result->lightbars);

                if (armor2ds.empty()) continue;
            }

            /// 2. Transform 2d to 3d
            /// @NOTE: 即将废弃，现在是 2d 观测直接输入 EKF 的时代了
            auto armor3ds = Armor3ds { };
            {
                estimator.update_camera_transform(context.camera_transform);

                auto result = estimator.estimate_armor(armor2ds, *image);

                const auto& addition = estimator.addition();
                visual.draw_later(addition.detected_2d);
                visual.draw_later(addition.areas);
                visual.draw_later(addition.predicted_near);
                visual.draw_later(addition.predicted_away);

                visual.publish(addition.origin, "origin_armors");
                visual.publish(addition.detected_3d, "outpost_lightbars");
                visual.publish({ addition.center_3d, Orientation::kIdentity() }, "outpost_center");

                armor3ds = std::move(result);
            }
            visual.publish(armor3ds, "visible_armors");

            /// @NOTE:
            ///  🚧 施工中...... 新的跟踪器即将到来
            ///  期待新的 EKF 的表现吧！
            auto trackable = Trackable::Unique { };
            {
                using namespace rmcs_msgs;
                if (context.id != RobotId::UNKNOWN) {
                    tracker_v2->update_track_color(
                        (context.id.color() == RobotColor::RED) ? CampColor::BLUE : CampColor::RED);
                }
                tracker_v2->update_track_genre(DeviceIds::Full());
                tracker_v2->update_camera(context.camera_transform);

                tracker_v2->clean();
                tracker_v2->store(armor2ds);
                tracker_v2->store(armor3ds);
                tracker_v2->store(lightbar2ds);

                trackable = tracker_v2->execute(image->get_timestamp());

                const auto& addition = tracker_v2->addition();
                for (const auto& item : addition.tracked2d) {
                    visual.draw_later(Canvas::ArmorShape {
                        .shape = item,
                        .color = { 127, 127, 127 },
                    });
                }
                visual.publish(addition.tracked3d, "trackable");
            }
            continue;

            /// 3. Apply Tracker
            auto target = tracker.decide(armor3ds, image->get_timestamp());
            auto cmd    = AutoAimState::kInvalid();
            if (target.snapshot) {
                auto& snapshot = target.snapshot;
                auto armors    = snapshot->predicted_armors(Clock::now());
                visual.publish(armors, "visible_robot");

                const auto gimbal_state = fire_control::GimbalState {
                    .timestamp = context.timestamp,
                    .yaw       = context.yaw,
                    .pitch     = context.pitch,
                };
                if (auto result = fire.solve(*snapshot, gimbal_state)) {
                    cmd.should_control = true;
                    cmd.target         = target.target_id;
                    cmd.should_shoot   = result->shoot_permitted;
                    cmd.yaw            = result->yaw;
                    cmd.pitch          = result->pitch;
                    cmd.robot_center   = result->center_position;

                    if (auto feedforward = result->feedforward) {
                        cmd.yaw_rate   = feedforward->yaw_rate;
                        cmd.pitch_rate = feedforward->pitch_rate;
                        cmd.yaw_acc    = feedforward->yaw_acc;
                        cmd.pitch_acc  = feedforward->pitch_acc;
                    }

                    auto impact_armors = snapshot->predicted_armors(result->impact_time);
                    visual.publish(impact_armors, "impact_robot");
                    visual.update_aiming_direction(cmd.yaw, cmd.pitch);
                    visual.update_mpc_plan(cmd.yaw, cmd.pitch, cmd.yaw_rate, cmd.pitch_rate,
                        cmd.yaw_acc, cmd.pitch_acc);

                    if (auto aim_2d = estimator.make_point2d(result->aim_point)) {
                        visual.draw_later(Canvas::Point {
                            .origin = aim_2d->make<cv::Point2i>(),
                            .radius = 5,
                            .color  = result->shoot_permitted ? kRed : kGreen,
                        });
                    }
                }
            }
            visual.draw_later(
                Canvas::Text { "ATTACK", { 10, 660 }, cmd.should_shoot ? kRed : kWhite });
            visual.draw_later(
                Canvas::Text { "CONTROL", { 10, 640 }, cmd.should_control ? kRed : kWhite });

            {
                std::lock_guard lock { self.command_mutex };
                self.current_command = cmd;
                self.unread_command.store(true, std::memory_order::release);
            }
        }
    }

    explicit Impl(AutoAim& self)
        : self { self } {

        const auto configs           = util::configs();
        const auto camera_matrix     = configs["camera_matrix"].as<std::array<double, 9>>();
        const auto distort_coeff     = configs["distort_coeff"].as<std::array<double, 5>>();
        const auto use_visualization = configs["use_visualization"].as<bool>();

        const auto handle_result = [&](auto runtime_name, const auto& result) {
            if (!result.has_value()) {
                node.error("Failed to init '{}'", runtime_name);
                node.error("  {}", result.error());
                util::panic(std::format("Failed to initialize {}", runtime_name));
            }
        };

        {
            auto config = configs["capturer"];
            handle_result("capturer", cap.initialize(config));
        }
        {
            auto config         = configs["identifier"];
            auto model_location = std::filesystem::path { util::Parameters::share_location() }
                / std::filesystem::path { config["model_location"].as<std::string>() };
            config["model_location"] = model_location.string();
            handle_result("identifier", identifier.initialize(config));
        }
        {
            auto config = configs["tracker"];
            handle_result("tracker", tracker.initialize(config));
        }
        {
            auto config = configs["pose_estimator"];
            handle_result("estimator", estimator.initialize(config));
            estimator.configure_camera(camera_matrix, distort_coeff);
        }
        {
            auto config = configs["fire_control"];
            handle_result("fire_control", fire.initialize(config));
        }
        if (use_visualization) {
            auto config = configs["visualization"];
            handle_result("visualization", visual.initialize(config));
        }

        tracker_v2 = std::make_unique<TrackerV2>(configs["tracker"]);
        tracker_v2->update_camera(camera_matrix);
        tracker_v2->update_camera(distort_coeff);

        worker = std::jthread([this](const std::stop_token& stop) { Impl::run(stop); });
    }
    ~Impl() {
        worker.request_stop();
        if (worker.joinable()) {
            worker.join();
        }
    }
};

AutoAim::AutoAim() noexcept
    : pimpl { std::make_unique<Impl>(*this) } { }

AutoAim::~AutoAim() noexcept = default;
