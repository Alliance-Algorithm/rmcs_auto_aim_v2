#include "kernel/capturer.hpp"
#include "utility/node.hpp"

#include <csignal>
#include <rclcpp/executors.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/utilities.hpp>

#include <hikcamera/capturer.hpp>

auto main(int argc, char* argv[]) -> int {
    using namespace rmcs;

    rclcpp::init(argc, argv);
    std::signal(SIGINT, [](auto) { rclcpp::shutdown(); });

    auto node = std::make_shared<utility::Node>("AutoAim");

    /// 1. Read image from capturer
    ///     Just one thread for receiving

    // auto capturer = std::make_unique<kernel::Capturer>();
    // if (auto result = capturer->initialize(); !result) {
    //     node->rclcpp_error("Failed to init capturer: {}", result.error());
    //     rclcpp::shutdown();
    // }
    // capturer->start_working();

    /// 2. Identify armors

    /// 3. Transform 2d to 3d

    /// 4. Update tracker

    /// 5. Solve tf and send command with fire controller

    rclcpp::spin(node);
    return rclcpp::shutdown();
}
