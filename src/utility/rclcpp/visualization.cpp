#include "visualization.hpp"
#include "utility/panic.hpp"
#include <rclcpp/node.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace rmcs::util {

using Marker = visualization_msgs::msg::Marker;

namespace visual {

    struct Context::Details {
        Marker marker_status;
        std::shared_ptr<rclcpp::Publisher<Marker>> rclcpp_pub;
    };
    auto Context::update() noexcept -> void {
        std::ignore = std::as_const(*this);
        details->rclcpp_pub->publish(details->marker_status);
    }

    Context::Context() noexcept
        : details { std::make_unique<Details>() } { }

    Context::~Context() noexcept = default;

    auto Armor::update() noexcept -> void { context.update(); }

    auto Armor::set_translation(const Translation& translation) noexcept -> void {
        std::ignore = std::as_const(*this);
        translation.copy_to(context.details->marker_status.pose.position);
    }
    auto Armor::set_orientation(const Orientation& orientation) noexcept -> void {
        std::ignore = std::as_const(*this);
        orientation.copy_to(context.details->marker_status.pose.orientation);
    }

}

struct Visualization::Impl {

    rclcpp::Node rclcpp;
    std::shared_ptr<rclcpp::Publisher<Marker>> pub;

    explicit Impl(const std::string& id) noexcept
        : rclcpp { id } { }

    auto bind_context(visual::Context& context) const noexcept -> void {
        context.details->rclcpp_pub = pub;
    }
};

auto Visualization::bind_context(visual::Context& context) noexcept -> void {
    pimpl->bind_context(context);
}

Visualization::Visualization(const std::string& id) noexcept
    : pimpl { std::make_unique<Impl>(id) } { }

Visualization::Visualization() noexcept { util::panic("Should not use this constructer!\n"); }

Visualization::~Visualization() noexcept = default;

}
