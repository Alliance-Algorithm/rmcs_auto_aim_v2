#include "transform.hpp"
#include "utility/panic.hpp"
#include "utility/rclcpp/node.details.hpp"

#include <cmath>
#include <format>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

using namespace rmcs::util::visual;

struct DynamicTransform::Impl {
    static inline rclcpp::Clock rclcpp_clock { RCL_STEADY_TIME };

    RclcppNode& node;
    std::string parent_frame;
    std::string child_frame;
    tf2_ros::TransformBroadcaster tf_broadcaster;

    explicit Impl(RclcppNode& node) noexcept
        : node { node }
        , tf_broadcaster { node.details->rclcpp } { }

    auto set_link(const std::string& parent, const std::string& child) -> void {
        if (!prefix::check_naming(parent) || !prefix::check_naming(child)) {
            util::panic(
                std::format("Invalid naming for transform frame: {}", prefix::naming_standard));
        }
        parent_frame = parent;
        child_frame  = child;
    }

    auto publish(const Transform& transform) -> void {
        if (std::isnan(transform.translation.x) || std::isnan(transform.translation.y)
            || std::isnan(transform.translation.z)) {
            return;
        }

        geometry_msgs::msg::TransformStamped msg;
        msg.header.stamp    = rclcpp_clock.now();
        msg.header.frame_id = parent_frame;
        msg.child_frame_id  = child_frame;

        transform.translation.copy_to(msg.transform.translation);
        transform.orientation.copy_to(msg.transform.rotation);

        tf_broadcaster.sendTransform(msg);
    }
};

DynamicTransform::DynamicTransform(RclcppNode& node)
    : pimpl { std::make_unique<Impl>(node) } { }

DynamicTransform::~DynamicTransform() noexcept = default;

auto DynamicTransform::set_link(const std::string& parent, const std::string& child) -> void {
    pimpl->set_link(parent, child);
}

auto DynamicTransform::publish(const Transform& transform) -> void { pimpl->publish(transform); }
