#include "posture.hpp"
#include "utility/panic.hpp"
#include "utility/rclcpp/node.details.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/publisher.hpp>

using namespace rmcs::util::visual;
using PoseStamped = geometry_msgs::msg::PoseStamped;

struct Posture::Impl {
    static inline rclcpp::Clock rclcpp_clock { RCL_SYSTEM_TIME };

    PoseStamped pose_stamped;
    rclcpp::Publisher<PoseStamped>::SharedPtr rclcpp_pub;

    explicit Impl(const Config& config) noexcept {
        if (!prefix::check_naming(config.id) || !prefix::check_naming(config.tf)) {
            util::panic(
                std::format("Not a valid naming for armor id or tf: {}", prefix::naming_standard));
        }
        auto& rclcpp = config.rclcpp.details->rclcpp;

        const auto topic_name { config.rclcpp.get_pub_topic_prefix() + config.id };
        rclcpp_pub = rclcpp->create_publisher<PoseStamped>(topic_name, qos::debug);

        pose_stamped.header.frame_id = config.tf;
    }
    auto move(const Translation& t, const Orientation& q) noexcept {
        t.copy_to(pose_stamped.pose.position);
        q.copy_to(pose_stamped.pose.orientation);
    }
    auto update() noexcept {
        pose_stamped.header.stamp = rclcpp_clock.now();
        rclcpp_pub->publish(pose_stamped);
    }
};

auto Posture::update() noexcept -> void { pimpl->update(); }

auto Posture::impl_move(const Translation& t, const Orientation& q) noexcept -> void {
    pimpl->move(t, q);
}

Posture::Posture(const Config& config) noexcept
    : pimpl { std::make_unique<Impl>(config) } { }

Posture::~Posture() noexcept = default;
