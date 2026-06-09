#pragma once

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
#include "utility/shared/context.hpp"
#include "utility/singleton/running.hpp"

#include <csignal>
#include <experimental/scope>
#include <filesystem>
#include <mutex>
#include <thread>

namespace rmcs {

class AutoAim {
private:
    util::RclcppNode node { "auto_aim" };

    kernel::Capturer cap { };
    kernel::Identifier identifier { };
    kernel::Tracker tracker { };
    kernel::PoseEstimator estimator { };
    kernel::FireControl fire { };
    kernel::Visualization visual { };

    std::jthread worker;

    std::mutex context_mutex;
    SystemContext current_context { SystemContext::kIdentity() };

    std::mutex command_mutex;
    std::atomic<bool> unread_command = false;
    AutoAimState current_command { AutoAimState::kInvalid() };

    auto run(const std::stop_token& stop) -> void {
        using namespace std::chrono_literals;

        node.set_pub_topic_prefix("/rmcs/auto_aim/");

        auto framerate = FramerateCounter { };
        framerate.set_interval(std::chrono::seconds { 5 });

        while (util::get_running() && !stop.stop_requested()) {
            node.spin_once();

            auto image = cap.fetch_image();
            if (!image) continue;

            auto context = SystemContext::kIdentity();
            {
                std::lock_guard lock(context_mutex);
                context = current_context;
            }
            visual.publish(context.camera_transform, "camera_link");

            if (framerate.tick()) {
                node.info("Autoaim Framerate: {}", framerate.fps());
            }

            [[maybe_unused]] auto streamer = std::experimental::scope_exit { [&] {
                visual.draw_later(
                    Text { cap.recording() ? "RECORD ON" : "RECORD OFF", { 640, 20 } });
                visual.update_image(*image);
            } };

            /// 1. Identify Armor
            auto armors_2d = Armor2ds { };
            {
                auto result = identifier.sync_identify(*image);
                if (!result.has_value()) continue;

                for (const auto& roi : result->areas) {
                    visual.draw_later(roi);
                }
                visual.draw_later(result->armors);
                visual.draw_later(result->green_light);

                using namespace rmcs_msgs;
                if (context.id != RobotId::UNKNOWN) {
                    if (context.id.color() == RobotColor::RED) {
                        tracker.set_enemy_color(CampColor::BLUE);
                    } else {
                        tracker.set_enemy_color(CampColor::RED);
                    }
                }

                tracker.set_invincible_armors(context.invincible_devices);
                armors_2d = tracker.filter_armors(result->armors);

                if (armors_2d.empty()) continue;
            }

            /// 2. Transform 2d to 3d
            auto armors_3d = Armor3ds { };
            {
                estimator.update_camera_transform(context.camera_transform);

                auto result = estimator.estimate_armor(armors_2d, *image);

                const auto& addition = estimator.addition();
                visual.draw_later(addition.detected_2d);
                visual.draw_later(addition.areas);
                visual.draw_later(addition.predicted_near);
                visual.draw_later(addition.predicted_away);

                visual.publish(addition.origin, "origin_armors");
                visual.publish(addition.detected_3d, "outpost_lightbars");
                visual.publish({ addition.center_3d, Orientation::kIdentity() }, "outpost_center");

                armors_3d = result;
                if (armors_3d.empty()) continue;

                visual.publish(armors_3d, "visible_armors");
            }

            /// 3. Apply Tracker
            auto target = tracker.decide(armors_3d, image->get_timestamp());
            auto cmd    = AutoAimState::kInvalid();
            if (target.snapshot) {
                auto& snapshot = target.snapshot;
                auto armors    = snapshot->predicted_armors(Clock::now());
                visual.publish(armors, "visible_robot");

                const auto yaw = context.yaw;
                if (auto result = fire.solve(*snapshot, yaw)) {
                    cmd.should_control    = true;
                    cmd.target            = target.target_id;
                    cmd.should_shoot      = result->shoot_permitted;
                    cmd.yaw               = result->yaw;
                    cmd.pitch             = result->pitch;
                    cmd.yaw_rate          = result->yaw_rate;
                    cmd.pitch_rate        = result->pitch_rate;
                    cmd.yaw_acc           = result->yaw_acc;
                    cmd.pitch_acc         = result->pitch_acc;
                    cmd.feedforward_valid = result->feedforward_valid;
                    cmd.robot_center      = result->center_position;

                    visual.update_aiming_direction(cmd.yaw, -cmd.pitch);
                    visual.update_mpc_plan(cmd.yaw, cmd.pitch, cmd.yaw_rate, cmd.pitch_rate,
                        cmd.yaw_acc, cmd.pitch_acc);
                }
            }

            {
                std::lock_guard lock(command_mutex);
                current_command = cmd;
                unread_command.store(true, std::memory_order::release);
            }
        }

        node.shutdown();
    }

public:
    AutoAim() {
        std::signal(SIGINT, [](int) { util::set_running(false); });

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
            auto config               = configs["identifier"];
            const auto model_location = std::filesystem::path { util::Parameters::share_location() }
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

        worker = std::jthread([this](const std::stop_token& stop) { run(stop); });
    }

    template <typename WithFunc>
        requires std::invocable<WithFunc, SystemContext&>
    auto with_context(WithFunc&& func) {
        std::lock_guard lock(context_mutex);
        func(current_context);
    }

    template <typename WithFunc>
        requires std::invocable<WithFunc, const AutoAimState&>
    auto with_command(WithFunc&& func) {
        std::lock_guard lock(command_mutex);
        func(current_command);
    }
    auto command_updated() -> bool {
        return unread_command.exchange(false, std::memory_order::acquire);
    }
};

}
