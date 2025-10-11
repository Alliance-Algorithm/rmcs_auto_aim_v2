#include "utility/node.hpp"

#include <boost/lockfree/spsc_queue.hpp>

#include <rclcpp/rate.hpp>
#include <rclcpp/utilities.hpp>

using namespace rmcs;
using boost::lockfree::capacity;
using boost::lockfree::spsc_queue;

auto main(int argc, char* argv[]) -> int {
    rclcpp::init(argc, argv);

    auto node = utility::Node { "auto_aim_runtime" };

    /// 1. Read image from capturer
    ///     Just one thread for receiving

    /// 2. Identify armors

    /// 3. Transform 2d to 3d

    /// 4. Update tracker

    /// 5. Solve tf and send command with fire controller
}
