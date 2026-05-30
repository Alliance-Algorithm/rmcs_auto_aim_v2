#include "lightbar.hpp"

#include "utility/panic.hpp"
#include "utility/rclcpp/node.details.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>

using namespace rmcs::util::visual;

using Marker = visualization_msgs::msg::Marker;

struct LightBar::Impl {
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
                "Not a valid naming for lightbar name or tf: {}", prefix::naming_standard));
        }

        marker.header.frame_id = config.tf;
        marker.ns              = config.name;
        marker.id              = config.id;
        marker.type            = Marker::LINE_LIST;
        marker.action          = Marker::ADD;
        marker.lifetime        = rclcpp::Duration::from_seconds(0.1);

        marker.scale.x = config.width;

        marker.color.r = config.r;
        marker.color.g = config.g;
        marker.color.b = config.b;
        marker.color.a = config.a;

        marker.points.resize(2);
    }

    auto set(const Point3d& top, const Point3d& bottom) noexcept -> void {
        marker.points[0].x = top.x;
        marker.points[0].y = top.y;
        marker.points[0].z = top.z;

        marker.points[1].x = bottom.x;
        marker.points[1].y = bottom.y;
        marker.points[1].z = bottom.z;
    }

    auto update() noexcept -> void {
        if (!rclcpp_pub) {
            rclcpp_pub = create_rclcpp_publisher(config);
        }

        marker.header.stamp = rclcpp_clock.now();
        rclcpp_pub->publish(marker);
    }
};

auto LightBar::set(const Point3d& top, const Point3d& bottom) noexcept -> void {
    pimpl->set(top, bottom);
}

auto LightBar::update() noexcept -> void { pimpl->update(); }

LightBar::LightBar(const Config& config) noexcept
    : pimpl { std::make_unique<Impl>(config) } { }

LightBar::~LightBar() noexcept = default;
