#include <rclcpp/executor.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>

const auto logger = rclcpp::get_logger("auto-aim-runtime");

/// @note 初步决定使用共享内存进行自瞄进程和组件进程的通讯
///       这两个进程解耦是势在必行的
auto main(int argc, const char* const* argv) -> int {
    rclcpp::init(argc, argv);

    RCLCPP_INFO(logger, "Hello World");

    rclcpp::shutdown();
}