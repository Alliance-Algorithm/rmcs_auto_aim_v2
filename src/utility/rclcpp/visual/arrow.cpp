#include "arrow.hpp"

#include "utility/panic.hpp"
#include "utility/rclcpp/node.details.hpp"

#include <visualization_msgs/msg/marker.hpp>

using namespace rmcs::util::visual;

using Marker = visualization_msgs::msg::Marker;

struct Arrow::Impl {
    static inline rclcpp::Clock rclcpp_clock { RCL_STEADY_TIME };

    Config config;

    Marker marker;
    std::shared_ptr<rclcpp::Publisher<Marker>> rclcpp_pub;

    explicit Impl(Config config) noexcept
        : config(std::move(config)) {
        initialize();
    }

    static auto create_rclcpp_publisher(Config const& config) noexcept
        -> std::shared_ptr<rclcpp::Publisher<Marker>> {
        const auto topic_name { config.rclcpp.get_pub_topic_prefix() + config.name };
        return config.rclcpp.details->make_pub<Marker>(topic_name, qos::debug);
    }

    auto initialize() noexcept -> void {
        if (!prefix::check_naming(config.name) || !prefix::check_naming(config.tf)) {
            util::panic(std::format(
                "Not a valid naming for arrow name or tf: {}", prefix::naming_standard));
        }

        marker.header.frame_id = config.tf;
        marker.ns              = config.name;
        marker.id              = config.id;
        marker.type            = Marker::ARROW;
        marker.action          = Marker::ADD;
        marker.lifetime        = rclcpp::Duration::from_seconds(0.1);

        marker.scale.x = config.length;
        marker.scale.y = config.width;
        marker.scale.z = config.height;

        marker.color.r = config.r;
        marker.color.g = config.g;
        marker.color.b = config.b;
        marker.color.a = config.a;
    }

    auto update() noexcept -> void {
        if (!rclcpp_pub) {
            rclcpp_pub = create_rclcpp_publisher(config);
        }

        marker.header.stamp = rclcpp_clock.now();
        rclcpp_pub->publish(marker);
    }

    auto move(const Translation& t, const Orientation& q) noexcept -> void {
        t.copy_to(marker.pose.position);
        q.copy_to(marker.pose.orientation);
    }
};

auto Arrow::update() noexcept -> void { pimpl->update(); }

auto Arrow::impl_move(const Translation& t, const Orientation& q) noexcept -> void {
    pimpl->move(t, q);
}

Arrow::Arrow(const Config& config) noexcept
    : pimpl { std::make_unique<Impl>(config) } { }

Arrow::~Arrow() noexcept = default;
