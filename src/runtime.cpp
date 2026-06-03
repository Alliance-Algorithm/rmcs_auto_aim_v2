#include "kernel/capturer.hpp"
#include "kernel/feishu.hpp"
#include "kernel/fire_control.hpp"
#include "kernel/identifier.hpp"
#include "kernel/pose_estimator.hpp"
#include "kernel/tracker.hpp"
#include "kernel/visualization.hpp"

#include "module/debug/framerate.hpp"
#include "utility/image/armor.hpp"
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
    auto configuration     = util::configuration();
    auto localhost_develop = configuration["localhost_develop"].as<bool>();
    auto use_visualization = configuration["use_visualization"].as<bool>();
    auto use_painted_image = configuration["use_painted_image"].as<bool>();
    auto use_painted_roi   = configuration["use_painted_roi"].as<bool>();

    auto handle_result = [&](auto runtime_name, const auto& result) {
        if (!result.has_value()) {
            node.error("Failed to init '{}'", runtime_name);
            node.error("  {}", result.error());
            util::panic(std::format("Failed to initialize {}", runtime_name));
        }
    };

    // CAPTURER
    {
        auto config = configuration["capturer"];
        auto result = capturer.initialize(config);
        handle_result("capturer", result);
    }
    // IDENTIFIER
    {
        auto config = configuration["identifier"];

        const auto model_location = std::filesystem::path { util::Parameters::share_location() }
            / std::filesystem::path { config["model_location"].as<std::string>() };
        config["model_location"] = model_location.string();

        auto result = identifier.initialize(config);
        handle_result("identifier", result);
    }
    // TRACKER
    {
        auto config = configuration["tracker"];
        auto result = tracker.initialize(config);
        handle_result("tracker", result);
    }
    // POSE ESTIMATOR
    {
        auto config = configuration["pose_estimator"];
        auto result = pose_estimator.initialize(config);
        handle_result("pose_estimator", result);
    }
    // FIRE CONTROL
    {
        auto config = configuration["fire_control"];
        auto result = fire_control.initialize(config);
        handle_result("fire_control", result);
    }
    // VISUALIZATION
    if (use_visualization) {
        auto config = configuration["visualization"];
        auto result = visualization.initialize(config, node);
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
            auto closest = feishu.search(image->get_timestamp(), 50ms);
            if (!closest) continue;

            context = *closest;
        }
        visualization.update_camera_pose(context.camera_transform.orientation);

        if (framerate.tick()) {
            node.info("Autoaim Framerate: {}", framerate.fps());
        }

        // 结束流程后发送串流帧
        [[maybe_unused]] auto _ =
            std::experimental::scope_exit { [&] { visualization.update_image(*image); } };

        /// 1. Identify Armor
        ///
        auto armors_2d = Armor2Ds { };
        {
            auto result = identifier.sync_identify(*image);
            if (!result.has_value()) continue; // 一般不会推理出错喵~

            using namespace rmcs_msgs;
            if (context.id == RobotId::UNKNOWN) {
                // Keep configured enemy color when self color is unknown.
            } else if (context.id.color() == RobotColor::RED)
                tracker.set_enemy_color(CampColor::BLUE);
            else if (context.id.color() == RobotColor::BLUE)
                tracker.set_enemy_color(CampColor::RED);

            tracker.set_invincible_armors(context.invincible_devices);
            armors_2d = tracker.filter_armors(result->armors);

            if (armors_2d.empty()) continue;
        }
        [[maybe_unused]] auto paint_image = std::experimental::scope_exit { [&] {
            if (use_painted_roi) {
                identifier.draw_green_light_roi(*image);
            }
            if (use_painted_image) {
                identifier.draw_green_light(*image);
                for (const auto& armor : armors_2d)
                    util::draw(*image, armor);
            }
        } };

        /// 2. Transform 2d to 3d
        ///
        auto armors_3d = Armor3Ds { };
        {
            pose_estimator.update_camera_transform(context.camera_transform);
            auto result = pose_estimator.estimate_armor(armors_2d, *image);
            pose_estimator.draw_debug(*image);
            pose_estimator.publish_debug();

            armors_3d = pose_estimator.into_odom_link(result);
            if (armors_3d.empty()) continue;

            visualization.update_visible_armors(armors_3d);
        }

        /// 3. Apply Tracker
        ///
        auto target  = tracker.decide(armors_3d, image->get_timestamp());
        auto command = AutoAimState::kInvalid();
        if (target.snapshot) {
            auto& snapshot = target.snapshot;
            auto armors    = snapshot->predicted_armors(Clock::now());
            visualization.update_visible_robot(armors);

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
