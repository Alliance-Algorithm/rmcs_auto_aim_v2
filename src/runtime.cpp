#include "kernel/capturer.hpp"
#include "kernel/feishu.hpp"
#include "kernel/fire_control.hpp"
#include "kernel/identifier.hpp"
#include "kernel/pose_estimator.hpp"
#include "kernel/tracker.hpp"
#include "kernel/visualization.hpp"

#include "module/debug/action_throttler.hpp"
#include "utility/image/armor.hpp"
#include "utility/panic.hpp"
#include "utility/rclcpp/configuration.hpp"
#include "utility/rclcpp/node.hpp"
#include "utility/rclcpp/parameters.hpp"
#include "utility/singleton/running.hpp"

#include <csignal>
#include <experimental/scope>
#include <string>
#include <string_view>
#include <yaml-cpp/yaml.h>

using namespace rmcs;
using namespace rmcs::util;
using namespace rmcs::kernel;

auto main() -> int {
    using namespace std::chrono_literals;

    std::signal(SIGINT, [](int) { util::set_running(false); });

    auto rclcpp_node = util::RclcppNode { "AutoAim" };
    rclcpp_node.set_pub_topic_prefix("/rmcs/auto_aim/");

    {
        /// Runtime
        auto feishu         = kernel::Feishu<AutoAimState, ControlState> { };
        auto capturer       = kernel::Capturer { };
        auto identifier     = kernel::Identifier { };
        auto tracker        = kernel::Tracker { };
        auto pose_estimator = kernel::PoseEstimator { };
        auto fire_control   = kernel::FireControl { };
        auto visualization  = kernel::Visualization { };

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
        constexpr auto identifier_failed_label { "identifier_failed" };
        {
            action_throttler.register_action(control_state_label, 1);
            action_throttler.register_action(identifier_failed_label, 1);
        }

        while (util::get_running()) {
            if (feishu.heartbeat()) {
                //
            }
            rclcpp_node.spin_once();

            if (auto image = capturer.fetch_image()) {
                auto stream_guard = std::experimental::scope_exit { [&] {
                    if (visualization.initialized()) {
                        visualization.send_image(*image);
                    }
                } };
                std::ignore = stream_guard;

                auto control_state = ControlState { };
                if (!is_local_runtime) {
                    if (feishu.updated()) {
                        action_throttler.reset(control_state_label);
                        feishu.recv([&](const auto& data) { control_state = data; });
                    } else {
                        action_throttler.dispatch(control_state_label,
                            [&] { rclcpp_node.warn("Control state 尚未更新，使用上一次缓存值."); });
                    }
                }
                auto next_state = AutoAimState { };
                next_state.reset();

                /// 1. Identify Armor
                ///
                auto armors_2d = Armor2Ds { };
                {
                    auto result = identifier.sync_identify(*image);
                    if (!result.has_value()) {
                        action_throttler.dispatch(identifier_failed_label,
                            [&] { rclcpp_node.error("Armor detection failed"); });
                    } else {
                        action_throttler.reset(identifier_failed_label);

                        tracker.set_invincible_armors(control_state.invincible_devices);
                        auto filtered = tracker.filter_armors(*result);
                        if (use_painted_image) {
                            for (const auto& armor_2d : filtered)
                                util::draw(*image, armor_2d);
                        }

                        armors_2d = std::move(filtered);
                    }
                }

                /// 2. Transform 2d to 3d
                ///
                auto armors_3d = Armor3Ds { };
                if (!armors_2d.empty()) {
                    auto solved_armors_3d = pose_estimator.solve_pnp(armors_2d);
                    if (solved_armors_3d && visualization.initialized()) {
                        std::ignore = visualization.solved_pnp_armors(*solved_armors_3d);
                    }

                    if (solved_armors_3d) {
                        pose_estimator.set_odom_to_camera_transform(
                            control_state.odom_to_camera_transform);
                        armors_3d = pose_estimator.odom_to_camera(*solved_armors_3d);
                    }
                }

                /// 3. Apply Tracker
                ///
                {
                    auto tracker_output = tracker.decide(armors_3d, image->get_timestamp());
                    auto tracked_target = tracker_output.target_id;
                    auto snapshot       = std::move(tracker_output.snapshot);

                    if (tracker_output.allow_takeover) {
                        next_state.set_hold_state(
                            control_state.yaw, control_state.pitch, tracked_target);
                    }

                    if (tracker_output.allow_takeover && snapshot) {
                        if (auto control_cmd = fire_control.solve(
                                *snapshot, tracker_output.tracking_confirmed, control_state.yaw)) {
                            next_state.set_tracking_state(control_cmd->yaw, control_cmd->pitch,
                                tracked_target, control_cmd->shoot_permitted);
                        }
                    }

                    if (visualization.initialized() && snapshot) {
                        visualization.predicted_armors(snapshot->predicted_armors(Clock::now()));
                    }
                }

                /// 4. Transmit State
                ///
                feishu.send([&](auto& data) { data = next_state; });
            }
        } // runtime loop scope
    } // runtime objects scope

    rclcpp_node.shutdown();
    return 0;
}
