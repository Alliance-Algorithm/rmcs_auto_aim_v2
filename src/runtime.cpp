#include "kernel/capturer.hpp"
#include "kernel/feishu.hpp"
#include "kernel/fire_control.hpp"
#include "kernel/identifier.hpp"
#include "kernel/pose_estimator.hpp"
#include "kernel/tracker.hpp"
#include "kernel/visualization.hpp"

#include "module/debug/action_throttler.hpp"
#include "module/debug/framerate.hpp"

#include "utility/image/armor.hpp"
#include "utility/panic.hpp"
#include "utility/rclcpp/configuration.hpp"
#include "utility/rclcpp/node.hpp"
#include "utility/rclcpp/parameters.hpp"
#include "utility/singleton/running.hpp"

#include <chrono>
#include <csignal>
#include <yaml-cpp/yaml.h>

using namespace rmcs;
using namespace rmcs::util;
using namespace rmcs::kernel;
using TrackerState = rmcs::tracker::State;

auto main() -> int {
    using namespace std::chrono_literals;

    std::signal(SIGINT, [](int) { util::set_running(false); });

    auto rclcpp_node = util::RclcppNode { "AutoAim" };
    rclcpp_node.set_pub_topic_prefix("/rmcs/auto_aim/");

    auto handle_result = [&](auto runtime_name, const auto& result) {
        if (!result.has_value()) {
            rclcpp_node.error("Failed to init '{}'", runtime_name);
            rclcpp_node.error("  {}", result.error());
            util::panic(std::format("Failed to initialize {}", runtime_name));
        }
    };

    auto framerate = FramerateCounter {};
    framerate.set_interval(5s);

    /// Runtime
    auto feishu         = kernel::Feishu<RuntimeRole::AutoAim> {};
    auto capturer       = kernel::Capturer {};
    auto identifier     = kernel::Identifier {};
    auto tracker        = kernel::Tracker {};
    auto pose_estimator = kernel::PoseEstimator {};
    auto fire_control   = kernel::FireControl {};
    auto visualization  = kernel::Visualization {};

    auto action_throttler = util::ActionThrottler { 1s, 233 };

    /// Configure
    auto configuration     = util::configuration();
    auto use_visualization = configuration["use_visualization"].as<bool>();
    auto use_painted_image = configuration["use_painted_image"].as<bool>();
    auto is_local_runtime  = configuration["is_local_runtime"].as<bool>();

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
        auto result = visualization.initialize(config, rclcpp_node);
        handle_result("visualization", result);
    }
    // DEBUG
    {
        action_throttler.register_action("control_state_not_updated", 3);
        action_throttler.register_action("tracker_tracking", 1);
        action_throttler.register_action("armor_not_detected");
        action_throttler.register_action("visualization_pnp_failed", 3);
        action_throttler.register_action("fire_control_failed");
        action_throttler.register_action("feishu_commit_failed");
    }

    // AUTO AIM RESULT
    auto auto_aim_state = AutoAimState {};

    for (;;) {
        if (!util::get_running()) [[unlikely]]
            break;

        rclcpp_node.spin_once();

        if (auto image = capturer.fetch_image()) {
            auto control_state = ControlState {};

            if (is_local_runtime) {
                control_state.set_identity();
                action_throttler.dispatch("control_state_not_updated",
                    [&] { rclcpp_node.info("在本机环境下运行，将Control State 设置为默认值"); });
            } else {
                if (!feishu.updated()) {
                    action_throttler.dispatch("control_state_not_updated",
                        [&] { rclcpp_node.warn("Control state尚未更新，使用上一次缓存值."); });
                } else {
                    action_throttler.reset("control_state_not_updated");
                }

                control_state = feishu.fetch();
            }

            auto armors_2d = identifier.sync_identify(*image);
            if (!armors_2d.has_value()) {
                action_throttler.dispatch(
                    "armor_not_detected", [&] { rclcpp_node.warn("未识别到装甲板"); });
                continue;
            }
            action_throttler.reset("armor_not_detected");

            tracker.set_invincible_armors(control_state.invincible_devices);
            auto filtered_armors_2d = tracker.filter_armors(*armors_2d);
            if (filtered_armors_2d.empty()) {
                continue;
            }

            if (use_painted_image) {
                for (const auto& armor_2d : filtered_armors_2d)
                    util::draw(*image, armor_2d);
            }

            if (visualization.initialized()) {
                visualization.send_image(*image);
            }

            auto armors_3d_opt = pose_estimator.solve_pnp(filtered_armors_2d);
            if (!armors_3d_opt.has_value()) {
                continue;
            }

            if (visualization.initialized()) {
                auto success = visualization.solved_pnp_armors(*armors_3d_opt);
                if (!success) {
                    action_throttler.dispatch("visualization_pnp_failed",
                        [&] { rclcpp_node.error("可视化PNP结算后的装甲板失败"); });
                } else {
                    action_throttler.reset("visualization_pnp_failed");
                }
            }

            pose_estimator.set_odom_to_camera_transform(control_state.odom_to_camera_transform);
            auto armors_3d = pose_estimator.odom_to_camera(*armors_3d_opt);

            auto [tracker_state, target_device, snapshot_opt] =
                tracker.decide(armors_3d, Clock::now());

            if (tracker_state == TrackerState::Tracking) {
                action_throttler.dispatch(
                    "tracker_tracking", [&] { rclcpp_node.info("已进入 Tracking 状态"); });
            } else {
                action_throttler.reset("tracker_tracking");
                continue;
            }

            if (!snapshot_opt) {
                continue;
            }

            auto const& snapshot = *snapshot_opt;

            fire_control.set_bullet_speed(control_state.bullet_speed);
            auto result_opt =
                fire_control.solve(snapshot, control_state.odom_to_muzzle_translation);
            if (!result_opt) {
                action_throttler.dispatch(
                    "fire_control_failed", [&] { rclcpp_node.warn("Fire control solve failed"); });
                continue;
            }
            action_throttler.reset("fire_control_failed");

            auto_aim_state.timestamp       = Clock::now();
            auto_aim_state.gimbal_takeover = true;
            auto_aim_state.shoot_permitted = true;
            auto_aim_state.yaw             = result_opt->yaw;
            auto_aim_state.pitch           = result_opt->pitch;

            {
                auto success = feishu.commit(auto_aim_state);
                if (!success) {
                    action_throttler.dispatch("feishu_commit_failed",
                        [&] { rclcpp_node.warn("Commit auto_aim_state failed"); });
                }
            }

            if (visualization.initialized()) {
                visualization.predicted_armors(snapshot.predicted_armors(Clock::now()));
            }

        } // image receive scope

    } // runtime loop scope

    rclcpp_node.shutdown();
}
