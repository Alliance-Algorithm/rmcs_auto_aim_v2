#include "config/config.hpp"
#include "kernel/capturer.hpp"
#include "utility/node.hpp"

#include <rclcpp/executors.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/utilities.hpp>

#include <csignal>

using namespace rmcs;

auto main(int argc, char* argv[]) -> int {
    std::signal(SIGINT, [](auto) { rclcpp::shutdown(); });

    rclcpp::init(argc, argv);

    auto node = std::make_shared<utility::Node>("AutoAim", utility::options);

    /// 0. Read config from yaml
    auto config = Config {};
    if (auto result = config.serialize("", *node); !result) {
        node->rclcpp_error("Failed to read yaml: {}", result.error());
        return rclcpp::shutdown();
    }

    auto capturer = std::make_unique<kernel::Capturer>();
    if (auto result = capturer->initialize(); !result) {
        node->rclcpp_error("Failed to init capturer: {}", result.error());
        return rclcpp::shutdown();
    }
    capturer->start_working();

    // other kernal module initialize
    // ...

    using namespace std::chrono_literals;
    node->create_wall_timer(1ms, [] {
        /// 1. Read image from capturer
        ///     Just one thread for receiving

        /// 2. Identify armors

        /// 3. Transform 2d to 3d

        /// 4. Update tracker

        /// 5. Solve tf and send command with fire controller
    });

    rclcpp::spin(node);
    return rclcpp::shutdown();
}
