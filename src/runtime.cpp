#include "kernel/capturer.hpp"
#include "kernel/control_system.hpp"
#include "kernel/identifier.hpp"
#include "kernel/pose_estimator.hpp"
#include "kernel/visualization.hpp"

#include "module/debug/framerate.hpp"
#include "utility/image/armor.hpp"
#include "utility/panic.hpp"
#include "utility/rclcpp/configuration.hpp"
#include "utility/rclcpp/node.hpp"
#include "utility/rclcpp/parameters.hpp"
#include "utility/singleton/running.hpp"

#include <csignal>
#include <yaml-cpp/yaml.h>

using namespace rmcs;

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
    auto capturer       = kernel::Capturer {};
    auto identifier     = kernel::Identifier {};
    auto pose_estimator = kernel::PoseEstimator {};
    auto visualization  = kernel::Visualization {};

    auto control_system = kernel::ControlSystem {};

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

            auto armors_2d = identifier.sync_identify(*image);
            if (!armors_2d.has_value()) {
                continue;
            }

            if (use_painted_image) {
                for (const auto& armor_2d : *armors_2d)
                    util::draw(*image, armor_2d);
            }

            if (visualization.initialized()) {
                visualization.send_image(*image);
            }

            using namespace rmcs::util;

            control_system.update_state({
                .timestamp = Clock::now(),
            });

            auto armors_3d = pose_estimator.solve_pnp(*armors_2d);

            if (!armors_3d.has_value()) continue;

            if (visualization.initialized()) {
                visualization.visualize_armors(*armors_3d);
            }
            // TODO: pose estimator
            // TODO: predictor
            // TODO: control
            rclcpp_node.spin_once();
        }
    }
    rclcpp_node.shutdown();
}
