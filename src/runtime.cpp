#include "kernel/kernel.hpp"
#include <csignal>
#include <rclcpp/executors.hpp>

using namespace rmcs;

auto main(int argc, char* argv[]) -> int {
    std::signal(SIGINT, [](auto) { rclcpp::shutdown(); });
    std::signal(SIGTERM, [](int) { rclcpp::shutdown(); });

    rclcpp::init(argc, argv);

    auto autoaim = std::make_shared<kernel::AutoAim>();
    try {
        rclcpp::spin(autoaim);
    } catch (const std::exception& e) {
        autoaim->error("Unexpected exception");
        autoaim->error("  e: {}", e.what());
    }
    return rclcpp::shutdown();
}
