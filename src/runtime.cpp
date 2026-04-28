#include "kernel/capturer.hpp"
#include "kernel/feishu.hpp"
#include "kernel/fire_control.hpp"
#include "kernel/identifier.hpp"
#include "kernel/pose_estimator.hpp"
#include "kernel/tracker.hpp"
#include "kernel/visualization.hpp"

#include "module/debug/framerate.hpp"
#include "utility/image/armor.hpp"
#include "utility/logging_util.hpp"
#include "utility/panic.hpp"
#include "utility/rclcpp/configuration.hpp"
#include "utility/rclcpp/node.hpp"
#include "utility/rclcpp/parameters.hpp"
#include "utility/singleton/running.hpp"

#include <csignal>
#include <experimental/scope>
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

    auto logging = LoggingUtil { node };
    logging.reset("receive", 5);
    logging.reset("detection", 5);

    /// Runtime
    auto feishu         = kernel::Feishu<AutoAimState, ControlState> { };
    auto capturer       = kernel::Capturer { };
    auto identifier     = kernel::Identifier { };
    auto tracker        = kernel::Tracker { };
    auto pose_estimator = kernel::PoseEstimator { };
    auto fire_control   = kernel::FireControl { };
    auto visualization  = kernel::Visualization { };

    /// Configure
    auto configuration     = util::configuration();
    auto use_visualization = configuration["use_visualization"].as<bool>();
    auto use_painted_image = configuration["use_painted_image"].as<bool>();
    auto without_rmcs      = configuration["is_local_runtime"].as<bool>();

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

        auto updated = feishu.heartbeat();

        auto image = capturer.fetch_image();
        if (!image) continue;

        if (framerate.tick()) {
            node.info("Autoaim Framerate: {}", framerate.fps());
        }

        // 结束流程后发送串流帧
        [[maybe_unused]] auto _ = std::experimental::scope_exit { [&] {
            if (visualization.initialized()) {
                visualization.send_image(*image);
            }
        } };

        auto received = ControlState::kInvalid();
        if (without_rmcs) {
            received = ControlState::kIdentity();
        }

        if (!updated) continue;
        received = *feishu.latest();

        /// 1. Identify Armor
        ///
        auto armors_2d = Armor2Ds { };
        {
            auto result = identifier.sync_identify(*image);
            if (!result.has_value()) {
                logging.error("detection", "Something wrong happend in identifier");
                continue; // 一般不会推理出错喵~
            }
            if (use_painted_image) {
                for (const auto& armor : *result)
                    util::draw(*image, armor);
            }
            logging.reset("detection", 5);

            tracker.set_invincible_armors(received.invincible_devices);
            armors_2d = tracker.filter_armors(*result);

            if (armors_2d.empty()) continue;
        }

        /// 2. Transform 2d to 3d
        ///
        auto armors_3d = Armor3Ds { };
        pose_estimator.update_camera_transform(received.odom_to_camera_transform);
        if (auto result = pose_estimator.solve_pnp(armors_2d)) {
            armors_3d = pose_estimator.odom_to_camera(*result);

            if (visualization.initialized()) {
                visualization.solved_pnp_armors(*result);
            }

            if (armors_3d.empty()) continue;
        }

        /// 3. Apply Tracker
        ///
        auto target    = tracker.decide(armors_3d, image->get_timestamp());
        auto target_id = target.target_id;

        auto command = AutoAimState::kInvalid();
        if (auto& snapshot = target.snapshot) {
            command.timestamp      = Clock::now();
            command.should_control = true;
            command.should_shoot   = false;
            command.yaw            = received.yaw;
            command.pitch          = received.pitch;
            command.target         = target_id;

            if (target.allow_control) {
                const auto control = target.tracking_confirmed;
                const auto yaw     = received.yaw;
                if (auto result = fire_control.solve(*snapshot, control, yaw)) {
                    command.should_shoot = result->shoot_permitted;
                    command.yaw          = result->yaw;
                    command.pitch        = result->pitch;
                }
            }

            if (visualization.initialized()) {
                visualization.predicted_armors(snapshot->predicted_armors(Clock::now()));
            }

            /// 4. Transmit State
            ///
            feishu.send(command);
        }

    } // runtime loop scope

    node.shutdown();
    return 0;
}
