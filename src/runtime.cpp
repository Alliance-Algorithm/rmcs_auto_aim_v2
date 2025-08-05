#include "modules/debug/visualization/visualization.hpp"
#include <opencv2/opencv.hpp>
#include <rclcpp/utilities.hpp>

int main(int argc, char** argv) {
    using namespace rmcs;

    rclcpp::init(argc, argv);

    auto node = utility::Node { "test" };
    module::Visualization::block_and_test(node);

    return rclcpp::shutdown();
}