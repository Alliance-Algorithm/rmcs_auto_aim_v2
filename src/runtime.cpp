#include "kernel/capturer.hpp"
#include "kernel/feishu.hpp"
#include "kernel/identifier.hpp"
#include "kernel/pose_estimator.hpp"
#include "kernel/tracker.hpp"
#include "kernel/visualization.hpp"

#include "module/debug/framerate.hpp"
#include "utility/image/armor.hpp"
#include "utility/logging/printer.hpp"
#include "utility/panic.hpp"
#include "utility/rclcpp/configuration.hpp"
#include "utility/rclcpp/node.hpp"
#include "utility/rclcpp/parameters.hpp"
#include "utility/singleton/running.hpp"

#include <csignal>
#include <yaml-cpp/yaml.h>

using namespace rmcs;
using namespace rmcs::util;
using TrackerState = rmcs::tracker::State;
using Clock        = std::chrono::steady_clock;

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
    ///
    auto feishu         = kernel::Feishu<AutoAimSide> {};
    auto capturer       = kernel::Capturer {};
    auto identifier     = kernel::Identifier {};
    auto tracker        = kernel::Tracker {};
    auto pose_estimator = kernel::PoseEstimator {};
    auto visualization  = kernel::Visualization {};
    auto log            = Printer { "runtime" };
    /// Configure
    ///
    auto configuration     = util::configuration();
    auto use_visualization = configuration["use_visualization"].as<bool>();
    auto use_painted_image = configuration["use_painted_image"].as<bool>();

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
    // VISUALIZATION
    if (use_visualization) {
        auto config = configuration["visualization"];
        auto result = visualization.initialize(config, rclcpp_node);
        handle_result("visualization", result);
    }

    for (;;) {
        if (!util::get_running()) [[unlikely]]
            break;

        if (auto image = capturer.fetch_image()) {
            auto control_state_opt = feishu.fetch<ControlState>();
            if (!control_state_opt) continue;
            auto& control_state = *control_state_opt;

            auto armors_2d = identifier.sync_identify(*image);
            if (!armors_2d.has_value()) {
                continue;
            }

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

            if (!armors_3d_opt.has_value()) continue;
            if (visualization.initialized()) {
                auto success = visualization.visualize_armors(*armors_3d_opt);
                if (!success) log.info("可视化装甲板失败");
            }

            pose_estimator.set_camera2world_transform(control_state.camera_to_odom_transform);
            auto armors_3d = pose_estimator.camera2world(*armors_3d_opt);

            auto [tracker_state, target_device, snapshot_opt] =
                tracker.decide(armors_3d, Clock::now());
            if (tracker_state != TrackerState::Tracking) continue;
            if (!snapshot_opt) continue;

            auto const& snapshot = *snapshot_opt;

            if (visualization.initialized()) {
                visualization.predicted_armors(snapshot, Clock::now());
            }

            rclcpp_node.spin_once();
        }
    }
    rclcpp_node.shutdown();
}
