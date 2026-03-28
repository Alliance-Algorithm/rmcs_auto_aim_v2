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

#include <chrono>
#include <cmath>
#include <csignal>
#include <experimental/scope>
#include <numbers>
#include <string>
#include <string_view>
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

    {
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
        constexpr auto identifier_failed_label { "identifier_failed" };
        constexpr auto visualization_pnp_label { "visualization_pnp_failed" };
        constexpr auto fire_control_label { "fire_control_failed" };
        constexpr auto feishu_commit_label { "feishu_commit_failed" };
        constexpr auto temporary_lost_label { "temporary_lost" };
        {
            action_throttler.register_action(control_state_label, 1);
            action_throttler.register_action(identifier_failed_label, 3);
            action_throttler.register_action(visualization_pnp_label, 3);
            action_throttler.register_action(fire_control_label);
            action_throttler.register_action(feishu_commit_label);
            action_throttler.register_action(temporary_lost_label);
        }

        ///
        /// Steps
        ///
        const auto fetch_control_state = [&] -> ControlState {
            if (is_local_runtime) {
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

        auto commit_sequence = 0ULL;
        const auto commit_state = [&](const AutoAimState& state, std::string_view reason) {
            const auto commit_seq          = ++commit_sequence;
            const auto cos_pitch          = std::cos(state.pitch);
            const auto target_direction_x = cos_pitch * std::cos(state.yaw);
            const auto target_direction_y = cos_pitch * std::sin(state.yaw);
            const auto target_direction_z = std::sin(state.pitch);
            constexpr auto rad_to_deg     = 180.0 / std::numbers::pi;
            const auto yaw_deg            = state.yaw * rad_to_deg;
            const auto pitch_deg          = state.pitch * rad_to_deg;
            constexpr auto abnormal_pitch_threshold_deg = 30.0;
            const auto vector_norm = std::sqrt(target_direction_x * target_direction_x
                + target_direction_y * target_direction_y + target_direction_z * target_direction_z);
            const auto state_age_ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - state.timestamp)
                    .count();
            const auto yaw_finite   = std::isfinite(state.yaw);
            const auto pitch_finite = std::isfinite(state.pitch);

            if (std::abs(pitch_deg) >= abnormal_pitch_threshold_deg) {
                rclcpp_node.warn(
                    "Abnormal control target vector=[{:.4f}, {:.4f}, {:.4f}] "
                    "(norm={:.4f}, yaw={:.2f}deg, pitch={:.2f}deg, threshold={:.2f}deg, "
                    "commit_seq={}, reason={}, target={}, gimbal_takeover={}, shoot_permitted={}, "
                    "yaw_finite={}, pitch_finite={}, state_age_ms={})",
                    target_direction_x, target_direction_y, target_direction_z, vector_norm, yaw_deg,
                    pitch_deg, abnormal_pitch_threshold_deg, commit_seq, reason,
                    rmcs::to_string(state.target), state.gimbal_takeover, state.shoot_permitted,
                    yaw_finite, pitch_finite, state_age_ms);
            }

            if (!feishu.commit(state)) {
                rclcpp_node.warn(
                    "Commit auto_aim_state failed, control_target_vector=[{:.4f}, {:.4f}, {:.4f}] "
                    "(commit_seq={}, reason={}, target={}, yaw={:.2f}deg, pitch={:.2f}deg)",
                    target_direction_x, target_direction_y, target_direction_z, commit_seq, reason,
                    rmcs::to_string(state.target), yaw_deg, pitch_deg);
                return false;
            }
            action_throttler.reset(feishu_commit_label);
            return true;
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
                auto armors_3d = Armor3Ds {};
                if (!armors_2d.empty()) {
                    auto solved_armors_3d = pose_estimator.solve_pnp(armors_2d);
                    if (solved_armors_3d && visualization.initialized()) {
                        auto success = visualization.solved_pnp_armors(*solved_armors_3d);
                        if (!success) {
                            action_throttler.dispatch(visualization_pnp_label,
                                [&] { rclcpp_node.error("可视化PNP结算后的装甲板失败"); });
                        } else {
                            action_throttler.reset(visualization_pnp_label);
                        }
                    }

                    if (!solved_armors_3d) {
                        auto state = AutoAimState {};
                        state.set_safe_state(control_state.yaw, control_state.pitch);
                        commit_state(state, "pnp_solve_failed_safe_state");
                        continue;
                    }

                    pose_estimator.set_odom_to_camera_transform(
                        control_state.odom_to_camera_transform);
                    armors_3d = pose_estimator.odom_to_camera(*solved_armors_3d);
                }

                /// 3. Apply Tracker
                ///
                auto snapshot       = std::optional<predictor::Snapshot> { std::nullopt };
                auto tracked_target = DeviceId::UNKNOWN;
                auto tracker_state  = TrackerState::Lost;
                auto temporary_lost = false;
                {
                    auto now       = Clock::now();
                    auto result    = tracker.decide(armors_3d, now);
                    tracker_state  = result.state;
                    tracked_target = result.target_id;
                    snapshot       = result.snapshot;
                    if (tracker_state == TrackerState::Lost
                        || tracker_state == TrackerState::Detecting || !snapshot) {
                        auto state = AutoAimState {};
                        state.set_safe_state(control_state.yaw, control_state.pitch);
                        commit_state(state, "tracker_lost_or_detecting_safe_state");
                        continue;
                    }
                    temporary_lost = (tracker_state == TrackerState::TemporaryLost);
                    // if (temporary_lost) {
                    //     action_throttler.dispatch(temporary_lost_label, [&] {
                    //         rclcpp_node.warn("Tracker temporary lost: keep predicting gimbal,
                    //         disable shooting");
                    //     });
                    // } else {
                    //     action_throttler.reset(temporary_lost_label);
                    // }
                }

                /// 4. Fire Control
                ///
                auto state            = AutoAimState {};
                state.timestamp       = Clock::now();
                state.gimbal_takeover = true;
                state.shoot_permitted = (tracker_state == TrackerState::Tracking);
                state.target          = tracked_target;

                auto control_cmd = std::optional<FireControl::Result> { std::nullopt };
                {
                    control_cmd = fire_control.solve(
                        *snapshot, Translation {}, state.shoot_permitted, control_state.yaw);
                    if (!control_cmd) {
                        action_throttler.dispatch(fire_control_label,
                            [&] { rclcpp_node.warn("Fire control solve failed"); });
                        auto safe_state = AutoAimState {};
                        safe_state.set_safe_state(control_state.yaw, control_state.pitch);
                        commit_state(safe_state, "fire_control_solve_failed_safe_state");
                        continue;
                    }
                }
                action_throttler.reset(fire_control_label);

                /// 5. Transmit State
                ///
                state.shoot_permitted = temporary_lost ? false : control_cmd->shoot_permitted;
                state.yaw             = control_cmd->yaw;
                state.pitch           = control_cmd->pitch;
                commit_state(state, "fire_control_result");

                if (visualization.initialized()) {
                    visualization.predicted_armors(snapshot->predicted_armors(Clock::now()));
                }
            }
        } // runtime loop scope
    } // runtime objects scope

    rclcpp_node.shutdown();
    return 0;
}
