#include "kernel/kernel.hpp"

#include <rclcpp/executors.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/utilities.hpp>

#include <csignal>

using namespace rmcs;

auto main(int argc, char* argv[]) -> int {
    std::signal(SIGINT, [](auto) { rclcpp::shutdown(); });
    std::signal(SIGKILL, [](auto) { std::exit(0); });

    rclcpp::init(argc, argv);

    auto node = std::make_shared<kernel::AutoAim>();

    /// 0. Read config from yaml

    /// 1. Read image from capturer

    /// 2. Identify armors

    /// 3. Transform 2d to 3d

    /// 4. Update tracker

    /// 5. Solve tf and send command with fire controller

    rclcpp::spin(node);

    return rclcpp::shutdown();
}
