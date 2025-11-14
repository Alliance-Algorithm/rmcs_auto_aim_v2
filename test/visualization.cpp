#include "utility/rclcpp/visualization.hpp"
#include <print>

auto main() -> int {
    std::println("> Hello World!!!");
    std::println("> Visualization Here using Rclcpp");

    auto a = rmcs::util::Visualization {};

    return 0;
}
