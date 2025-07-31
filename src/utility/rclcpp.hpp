#pragma once
#include <rclcpp/node.hpp>

namespace rmcs::utility {

struct Node : public rclcpp::Node {
    using rclcpp::Node::Node;

    template <typename... Args>
    auto rclcpp_info(std::format_string<Args...> fmt, Args&&... args) const noexcept -> void {
        RCLCPP_INFO(get_logger(), "%s", std::format(fmt, std::forward<Args>(args)...).c_str());
    }
    template <typename... Args>
    auto rclcpp_warn(std::format_string<Args...> fmt, Args&&... args) const noexcept -> void {
        RCLCPP_WARN(get_logger(), "%s", std::format(fmt, std::forward<Args>(args)...).c_str());
    }
    template <typename... Args>
    auto rclcpp_error(std::format_string<Args...> fmt, Args&&... args) const noexcept -> void {
        RCLCPP_ERROR(get_logger(), "%s", std::format(fmt, std::forward<Args>(args)...).c_str());
    }
};

inline auto options = rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true);

} // namespace rmcs::utility
