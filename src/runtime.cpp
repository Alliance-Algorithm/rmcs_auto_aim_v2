#include "config/config.hpp"
#include "utility/node.hpp"

#include <rclcpp/executors.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/utilities.hpp>

#include <csignal>

using namespace rmcs;

auto main(int argc, char* argv[]) -> int {
    std::signal(SIGINT, [](auto) { rclcpp::shutdown(); });

    rclcpp::init(argc, argv);

    auto node = std::make_shared<util::Node>("AutoAim", util::options);

    /// 0. Read config from yaml
    auto config = Config {};
    if (auto result = config.serialize("", *node); !result) {
        node->rclcpp_error("Failed to read yaml: {}", result.error());
        return rclcpp::shutdown();
    }

    // other kernal module initialize
    // ...

    /// 1. Read image from capturer

    /// 2. Identify armors

    /// 3. Transform 2d to 3d

    /// 4. Update tracker

    /// 5. Solve tf and send command with fire controller

    return rclcpp::shutdown();
}
