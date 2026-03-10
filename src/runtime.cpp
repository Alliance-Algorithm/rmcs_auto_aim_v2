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
#include <experimental/scope>
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

    auto handle_result = [&](auto runtime_name, const auto& result) {
        if (!result.has_value()) {
            rclcpp_node.error("Failed to init '{}'", runtime_name);
            rclcpp_node.error("  {}", result.error());
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
        auto result = visualization.initialize(config, rclcpp_node);
        handle_result("visualization", result);
    }
    // DEBUG
    constexpr auto control_state_label { "control_state_not_updated" };
    constexpr auto tracker_tracking_label { "tracker_tracking" };
    constexpr auto armor_detected_label { "armor_not_detected" };
    constexpr auto visualization_pnp_label { "visualization_pnp_failed" };
    constexpr auto fire_control_label { "fire_control_failed" };
    constexpr auto feishu_commit_label { "feishu_commit_failed" };
    {
        action_throttler.register_action(control_state_label, 1);
        action_throttler.register_action(tracker_tracking_label, 1);
        action_throttler.register_action(armor_detected_label);
        action_throttler.register_action(visualization_pnp_label, 3);
        action_throttler.register_action(fire_control_label);
        action_throttler.register_action(feishu_commit_label);
    }

    ///
    /// Steps
    ///
    const auto fetch_control_state = [&] -> ControlState {
        if (is_local_runtime) {
            action_throttler.dispatch(control_state_label,
                [&] { rclcpp_node.info("在本机环境下运行，将Control State 设置为默认值"); });
            auto state = ControlState {};
            state.set_identity();
            return state;
        }
        if (!feishu.updated()) {
            action_throttler.dispatch(control_state_label,
                [&] { rclcpp_node.warn("Control state 尚未更新，使用上一次缓存值."); });
        } else {
            action_throttler.reset(control_state_label);
        }
        return feishu.fetch();
    };

    for (;;) {
        if (!util::get_running()) [[unlikely]]
            break;

        rclcpp_node.spin_once();

        if (auto image = capturer.fetch_image()) {
            auto stream_guard = std::experimental::scope_exit { [&] {
                if (visualization.initialized()) {
                    visualization.send_image(*image);
                }
            } };
            std::ignore = stream_guard;

            auto control_state = fetch_control_state();

            /// 1. Identify Armor
            ///
            auto armors_2d = Armor2Ds {};
            {
                auto result = identifier.sync_identify(*image);
                if (!result.has_value()) {
                    action_throttler.dispatch(
                        armor_detected_label, [&] { rclcpp_node.warn("未识别到装甲板"); });
                    continue;
                }
                action_throttler.reset(armor_detected_label);

                tracker.set_invincible_armors(control_state.invincible_devices);
                auto filtered = tracker.filter_armors(*result);
                // No available armors to shoot
                if (filtered.empty()) continue;

                if (use_painted_image) {
                    for (const auto& armor_2d : filtered)
                        util::draw(*image, armor_2d);
                }
                armors_2d = std::move(filtered);
            }

            /// 2. Transform 2d to 3d
            ///
            auto armors_3d = pose_estimator.solve_pnp(armors_2d);
            if (armors_3d && visualization.initialized()) {
                auto success = visualization.solved_pnp_armors(*armors_3d);
                if (!success) {
                    action_throttler.dispatch(visualization_pnp_label,
                        [&] { rclcpp_node.error("可视化PNP结算后的装甲板失败"); });
                } else {
                    action_throttler.reset(visualization_pnp_label);
                }
            }
            if (armors_3d) {
                auto transform = control_state.odom_to_camera_transform;
                pose_estimator.set_odom_to_camera_transform(transform);

                armors_3d = pose_estimator.odom_to_camera(*armors_3d);
            } else {
                continue;
            }

            /// 3. Apply Tracker
            ///
            auto snapshot = std::optional<predictor::Snapshot> { std::nullopt };
            {
                auto result = tracker.decide(*armors_3d, Clock::now());

                if (result.state == TrackerState::Tracking) {
                    action_throttler.dispatch(
                        tracker_tracking_label, [&] { rclcpp_node.info("已进入 Tracking 状态"); });
                } else {
                    action_throttler.reset(tracker_tracking_label);
                    continue;
                }

                snapshot = result.snapshot;
                if (!snapshot) continue;
            }

            /// 4. Fire Control
            ///
            auto control_cmd = std::optional<FireControl::Result> { std::nullopt };
            {
                fire_control.set_bullet_speed(control_state.bullet_speed);
                auto translation = control_state.odom_to_muzzle_translation;

                control_cmd = fire_control.solve(*snapshot, translation);
                if (!control_cmd) {
                    action_throttler.dispatch(
                        fire_control_label, [&] { rclcpp_node.warn("Fire control solve failed"); });
                    continue;
                } else {
                    action_throttler.reset(fire_control_label);
                }
            }

            /// 5. Transmit State
            ///
            auto state            = AutoAimState {};
            state.timestamp       = Clock::now();
            state.gimbal_takeover = true;
            state.shoot_permitted = true;
            state.yaw             = control_cmd->yaw;
            state.pitch           = control_cmd->pitch;
            if (!feishu.commit(state)) {
                action_throttler.dispatch(
                    feishu_commit_label, [&] { rclcpp_node.warn("Commit auto_aim_state failed"); });
            }

            if (visualization.initialized()) {
                visualization.predicted_armors(snapshot->predicted_armors(Clock::now()));
            }

        } // image receive scope

    } // runtime loop scope

    rclcpp_node.shutdown();
}
