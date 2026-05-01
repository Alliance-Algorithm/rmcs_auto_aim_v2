#include "mpc_plan_visualizer.hpp"

#include "utility/rclcpp/node.details.hpp"

#include <functional>
#include <memory>
#include <optional>

#include <geometry_msgs/msg/vector3_stamped.hpp>

using namespace rmcs::debug;
using namespace rmcs::util;

using DebugVector3 = geometry_msgs::msg::Vector3Stamped;

struct MpcPlanVisualizer::Impl {
    static inline rclcpp::Clock rclcpp_clock { RCL_STEADY_TIME };

    std::optional<std::reference_wrapper<RclcppNode>> node;

    std::shared_ptr<rclcpp::Publisher<DebugVector3>> planned_yaw_pub;
    std::shared_ptr<rclcpp::Publisher<DebugVector3>> planned_pitch_pub;

    auto initialize(RclcppNode& visual_node) noexcept -> void {
        node = std::ref(visual_node);

        const auto topic_prefix = visual_node.get_pub_topic_prefix();
        planned_yaw_pub =
            visual_node.details->make_pub<DebugVector3>(topic_prefix + "planned_yaw", qos::debug);
        planned_pitch_pub = visual_node.details->make_pub<DebugVector3>(
            topic_prefix + "planned_pitch", qos::debug);
    }

    auto publish(
        const std::shared_ptr<rclcpp::Publisher<DebugVector3>>& publisher, double x, double y,
        double z) const noexcept -> bool {
        if (!node.has_value()) return false;
        if (!publisher) return false;

        auto message          = DebugVector3 { };
        message.header.stamp  = rclcpp_clock.now();
        message.header.frame_id = "mpc_plan";
        message.vector.x      = x;
        message.vector.y      = y;
        message.vector.z      = z;
        publisher->publish(message);
        return true;
    }

    auto publish_planned_yaw(double yaw, double yaw_rate, double yaw_acc) const noexcept -> bool {
        return publish(planned_yaw_pub, yaw, yaw_rate, yaw_acc);
    }

    auto publish_planned_pitch(double pitch, double pitch_rate, double pitch_acc) const noexcept
        -> bool {
        return publish(planned_pitch_pub, pitch, pitch_rate, pitch_acc);
    }
};

auto MpcPlanVisualizer::initialize(RclcppNode& node) noexcept -> void {
    pimpl->initialize(node);
}

auto MpcPlanVisualizer::publish_planned_yaw(double yaw, double yaw_rate, double yaw_acc) const
    noexcept -> bool {
    return pimpl->publish_planned_yaw(yaw, yaw_rate, yaw_acc);
}

auto MpcPlanVisualizer::publish_planned_pitch(double pitch, double pitch_rate, double pitch_acc)
    const noexcept -> bool {
    return pimpl->publish_planned_pitch(pitch, pitch_rate, pitch_acc);
}

MpcPlanVisualizer::MpcPlanVisualizer() noexcept
    : pimpl { std::make_unique<Impl>() } { }

MpcPlanVisualizer::~MpcPlanVisualizer() noexcept = default;
