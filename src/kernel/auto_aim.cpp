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
    PoseEstimator estimator { };
    Visualization visual { };

    std::unique_ptr<TrackerV2> tracker_v2;
    std::unique_ptr<FireControllerV2> fire_v2;

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

            auto timestamp = Timestamp { };
            auto yaw       = 0.0;
            auto pitch     = 0.0;
            auto transform = Transform::kIdentity();
            auto id        = rmcs_msgs::RobotId { };
            {
                std::lock_guard lock { self.context_mutex };

                timestamp = self.current_context.timestamp;
                yaw       = self.current_context.yaw;
                pitch     = self.current_context.pitch;
                transform = self.current_context.camera_transform;
                id        = self.current_context.id;
            }
            visual.publish(transform, "camera_link");
            visual.publish(yaw, "yaw");

            if (framerate.tick()) {
                node.info("Autoaim Framerate: {}", framerate.fps());
            }

            [[maybe_unused]] auto streamer = std::experimental::scope_exit { [&] {
                visual.draw_later( // 录制开关
                    Canvas::Text { cap.recording() ? "RECORD ON" : "RECORD OFF", { 10, 700 } });
                visual.draw_later( // 自瞄帧率
                    Canvas::Text { std::format("FPS: {}", framerate.fps()), { 10, 680 } });
                visual.update_image(image->mat);
            } };

            /// 1. Identify Armor
            auto armor2ds    = Armor2ds { };
            auto lightbar2ds = Lightbar2ds { };
            if (auto result = identifier.sync_identify(image->mat)) {

                for (const auto& roi : result->areas) {
                    visual.draw_later(roi);
                }
                visual.draw_later(result->armors);
                visual.draw_later(result->green_light);

                armor2ds    = std::move(result->armors);
                lightbar2ds = std::move(result->lightbars);
            }

            /// 2. Transform 2d to 3d
            auto armor3ds = Armor3ds { };
            {
                estimator.update_camera_transform(transform);

                auto result = estimator.estimate_armor(armor2ds, image->mat);

                const auto& addition = estimator.addition();
                visual.draw_later(addition.detected_2d);
                // visual.draw_later(addition.areas);
                // visual.draw_later(addition.predicted_near);
                // visual.draw_later(addition.predicted_away);

                visual.publish(addition.origin, "origin_armors");
                visual.publish(addition.detected_3d, "outpost_lightbars");
                visual.publish({ addition.center_3d, Orientation::kIdentity() }, "outpost_center");

                armor3ds = std::move(result);
            }
            visual.publish(armor3ds, "visible_armors");

            auto trackable = Trackable::Unique { };
            {
                using namespace rmcs_msgs;
                if (id != RobotId::UNKNOWN) {
                    tracker_v2->update_track_color(
                        (id.color() == RobotColor::RED) ? CampColor::BLUE : CampColor::RED);
                }
                tracker_v2->update_track_genre(DeviceIds::Full());
                tracker_v2->update_camera(transform);

                tracker_v2->clean();
                tracker_v2->store(armor2ds);
                tracker_v2->store(armor3ds);
                tracker_v2->store(lightbar2ds);

                trackable = tracker_v2->execute(image->timestamp);

                const auto& addition = tracker_v2->addition();
                for (const auto& item : addition.tracked2d) {
                    visual.draw_later(Canvas::ArmorShape {
                        .shape = item,
                        .color = { 127, 127, 127 },
                    });
                }
                for (const auto& item : addition.lightbars) {
                    visual.draw_later(Canvas::Text {
                        .content  = std::to_string(item.id),
                        .top_left = item.point.make<cv::Point2i>(),
                        .color    = kYellow,
                    });
                }
                for (const auto& info : addition.infos) {
                    if (auto p = estimator.make_point2d(info.point)) {
                        visual.draw_later(Canvas::Text {
                            .content  = info.text,
                            .top_left = p->make<cv::Point2i>(),
                        });
                    }
                }
                visual.publish(addition.tracked3d, "trackable");
            }

            /// 3. 弹道解算
            auto cmd = Command::kInvalid();
            if (trackable) {
                fire_v2->update(timestamp, yaw, pitch);

                if (auto aimed = fire_v2->aim(*trackable)) {
                    cmd.should_track = true;
                    cmd.should_shoot = aimed->shoot;
                    cmd.yaw          = aimed->yaw;
                    cmd.pitch        = aimed->pitch;
                    cmd.robot_center = aimed->center;

                    visual.update_aiming_direction(cmd.yaw, cmd.pitch);
                    visual.publish(aimed->yaw, "aim_yaw");

                    if (auto aim_2d = estimator.make_point2d(aimed->attack)) {
                        const auto color = aimed->shoot ? kRed : aimed->pre_aim ? kYellow : kGreen;
                        visual.draw_later(Canvas::Point {
                            .origin = aim_2d->make<cv::Point2i>(),
                            .radius = 3,
                            .color  = color,
                        });
                    }
                    visual.draw_later(
                        Canvas::Text { "PREAIM", { 10, 620 }, aimed->pre_aim ? kRed : kWhite });
                }
            }
            visual.draw_later(
                Canvas::Text { "SHOOT", { 10, 660 }, cmd.should_shoot ? kRed : kWhite });
            visual.draw_later(
                Canvas::Text { "TRACK", { 10, 640 }, cmd.should_track ? kRed : kWhite });

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
            auto config = configs["pose_estimator"];
            handle_result("estimator", estimator.initialize(config));
            estimator.configure_camera(camera_matrix, distort_coeff);
        }
        if (use_visualization) {
            auto config = configs["visualization"];
            handle_result("visualization", visual.initialize(config));
        }

        tracker_v2 = std::make_unique<TrackerV2>(configs["tracker"]);
        tracker_v2->update_camera(camera_matrix);
        tracker_v2->update_camera(distort_coeff);

        fire_v2 = std::make_unique<FireControllerV2>(configs["fire_control"]);

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
