#include "kernel/capturer.hpp"
#include "kernel/feishu.hpp"
#include "kernel/fire_control.hpp"
#include "kernel/identifier.hpp"
#include "kernel/pose_estimator.hpp"
#include "kernel/tracker.hpp"
#include "kernel/visualization.hpp"

#include "utility/framerate.hpp"
#include "utility/image/text.hpp"
#include "utility/math/linear.hpp"
#include "utility/panic.hpp"
#include "utility/rclcpp/configuration.hpp"
#include "utility/rclcpp/node.hpp"
#include "utility/rclcpp/parameters.hpp"
#include "utility/shared/context.hpp"
#include "utility/singleton/running.hpp"

#include <chrono>
#include <csignal>
#include <experimental/scope>
#include <filesystem>
#include <string>
#include <yaml-cpp/yaml.h>

using namespace rmcs;
using namespace rmcs::util;
using namespace rmcs::kernel;

auto main() -> int {
    using namespace std::chrono_literals;

    std::signal(SIGINT, [](int) { util::set_running(false); });

    auto node = RclcppNode { "AutoAim" };
    node.set_pub_topic_prefix("/rmcs/auto_aim/");

    /// Runtime
    auto feishu         = kernel::Feishu<AutoAimState, SystemContext> { };
    auto capturer       = kernel::Capturer { };
    auto identifier     = kernel::Identifier { };
    auto tracker        = kernel::Tracker { };
    auto pose_estimator = kernel::PoseEstimator { };
    auto fire_control   = kernel::FireControl { };
    auto visualization  = kernel::Visualization { };

    /// Configure
    const auto configs           = util::configs();
    const auto localhost_develop = configs["localhost_develop"].as<bool>();
    const auto use_visualization = configs["use_visualization"].as<bool>();

    const auto camera_matrix = configs["camera_matrix"].as<std::array<double, 9>>();
    const auto distort_coeff = configs["distort_coeff"].as<std::array<double, 5>>();

    const auto handle_result = [&](auto runtime_name, const auto& result) {
        if (!result.has_value()) {
            node.error("Failed to init '{}'", runtime_name);
            node.error("  {}", result.error());
            util::panic(std::format("Failed to initialize {}", runtime_name));
        }
    };

    // CAPTURER
    {
        auto config = configs["capturer"];
        auto result = capturer.initialize(config);
        handle_result("capturer", result);
    }
    // IDENTIFIER
    {
        auto config = configs["identifier"];

        const auto model_location = std::filesystem::path { util::Parameters::share_location() }
            / std::filesystem::path { config["model_location"].as<std::string>() };
        config["model_location"] = model_location.string();

        auto result = identifier.initialize(config);
        handle_result("identifier", result);
    }
    // TRACKER
    {
        auto config = configs["tracker"];
        auto result = tracker.initialize(config);
        handle_result("tracker", result);
    }
    // POSE ESTIMATOR
    {
        auto config = configs["pose_estimator"];
        auto result = pose_estimator.initialize(config);
        handle_result("pose_estimator", result);
        pose_estimator.configure_camera(camera_matrix, distort_coeff);
    }
    // FIRE CONTROL
    {
        auto config = configs["fire_control"];
        auto result = fire_control.initialize(config);
        handle_result("fire_control", result);
    }
    // VISUALIZATION
    if (use_visualization) {
        auto config = configs["visualization"];
        auto result = visualization.initialize(config);
        handle_result("visualization", result);
    }

    auto framerate = FramerateCounter { };
    framerate.set_interval(std::chrono::seconds { 5 });

    while (util::get_running()) {
        node.spin_once();

        if (!localhost_develop && !feishu.heartbeat()) continue;

        auto image = capturer.fetch_image();
        if (!image) continue;

        auto context = SystemContext::kIdentity();
        if (!localhost_develop) {
            using namespace std::chrono_literals;
            const auto timestamp = image->get_timestamp() - 8'200'000ns;
            const auto closest   = feishu.search(timestamp, 50ms);
            if (!closest) continue;

            context = *closest;
        }
        visualization.publish_odom(context.camera_transform, "camera_link");

        if (framerate.tick()) {
            node.info("Autoaim Framerate: {}", framerate.fps());
        }

        // 结束流程后发送串流帧
        [[maybe_unused]] auto streamer =
            std::experimental::scope_exit { [&] { visualization.update_image(*image); } };

        /// 1. Identify Armor
        ///
        auto armors_2d = Armor2ds { };
        {
            auto result = identifier.sync_identify(*image);
            if (!result.has_value()) continue; // 一般不会推理出错喵~

            for (const auto& roi : result->areas) {
                visualization.draw_later(roi);
            }
            visualization.draw_later(result->armors);
            visualization.draw_later(result->green_light);

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
        ///
        auto armors_3d = Armor3ds { };
        {
            pose_estimator.update_camera_transform(context.camera_transform);

            auto result = pose_estimator.estimate_armor(armors_2d, *image);

            const auto& addition = pose_estimator.addition();
            visualization.draw_later(addition.detected_2d);
            visualization.draw_later(addition.areas);
            visualization.draw_later(addition.predicted_near);
            visualization.draw_later(addition.predicted_away);
            visualization.publish(addition.origin, "origin_armors");
            visualization.publish(addition.detected_3d, "outpost_lightbars");

            auto center_transform = Transform { addition.center_3d, Orientation::kIdentity() };
            visualization.publish_odom(center_transform, "center_3d");

            armors_3d = result;
            if (armors_3d.empty()) continue;

            visualization.publish(armors_3d, "visible_armors");
        }

        /// 3. Apply Tracker
        ///
        auto target  = tracker.decide(armors_3d, image->get_timestamp());
        auto command = AutoAimState::kInvalid();
        if (target.snapshot) {
            auto& snapshot = target.snapshot;
            auto armors    = snapshot->predicted_armors(Clock::now());
            visualization.publish(armors, "visible_robot");

            const auto yaw = context.yaw;
            if (auto result = fire_control.solve(*snapshot, yaw)) {
                command.should_control    = true;
                command.target            = target.target_id;
                command.should_shoot      = result->shoot_permitted;
                command.yaw               = result->yaw;
                command.pitch             = result->pitch;
                command.yaw_rate          = result->yaw_rate;
                command.pitch_rate        = result->pitch_rate;
                command.yaw_acc           = result->yaw_acc;
                command.pitch_acc         = result->pitch_acc;
                command.feedforward_valid = result->feedforward_valid;
                command.robot_center      = result->center_position;

                visualization.update_aiming_direction(command.yaw, -command.pitch);

                // TODO: 将 MPC 的 target 曲线和 Yaw 置于一个统一的参考系
                visualization.update_mpc_plan(command.yaw, command.pitch, command.yaw_rate,
                    command.pitch_rate, command.yaw_acc, command.pitch_acc);
            }
        }
        if (use_visualization) {
            util::draw_text(*image, command.should_shoot ? "ATTACK" : "IDLE");
        }

        /// 4. Transmit State
        ///
        feishu.send(command);

    } // runtime loop scope

    node.shutdown();
    return 0;
}
