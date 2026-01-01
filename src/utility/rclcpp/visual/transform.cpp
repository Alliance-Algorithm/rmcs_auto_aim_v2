#include "transform.hpp"
#include "utility/panic.hpp"
#include "utility/rclcpp/node.details.hpp"

#include <format>
#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace rmcs::util::visual;
using TransformMsg = geometry_msgs::msg::TransformStamped;

struct Transform::Impl {
    static inline rclcpp::Clock rclcpp_clock { RCL_STEADY_TIME };

    TransformMsg msg;
    rclcpp::Publisher<TransformMsg>::SharedPtr rclcpp_pub;

    explicit Impl(const Config& config) {
        if (!prefix::check_naming(config.topic) || !prefix::check_naming(config.parent_frame)
            || !prefix::check_naming(config.child_frame)) {
            util::panic(std::format(
                "Invalid naming for transform topic/frame: {}", prefix::naming_standard));
        }

        auto& rclcpp = config.rclcpp.details->rclcpp;
        const auto topic_name { config.rclcpp.get_pub_topic_prefix() + config.topic };
        rclcpp_pub = rclcpp->create_publisher<TransformMsg>(topic_name, qos::debug);

        msg.header.frame_id = config.parent_frame;
        msg.child_frame_id  = config.child_frame;
    }

    auto move(const Translation& t, const Orientation& q) noexcept {
        t.copy_to(msg.transform.translation);
        q.copy_to(msg.transform.rotation);
    }

    auto update() noexcept {
        msg.header.stamp = rclcpp_clock.now();
        rclcpp_pub->publish(msg);
    }
};

auto Transform::update() noexcept -> void { pimpl->update(); }

auto Transform::impl_move(const Translation& t, const Orientation& q) noexcept -> void {
    pimpl->move(t, q);
}

Transform::Transform(const Config& config) noexcept
    : pimpl { std::make_unique<Impl>(config) } { }

Transform::~Transform() noexcept = default;
