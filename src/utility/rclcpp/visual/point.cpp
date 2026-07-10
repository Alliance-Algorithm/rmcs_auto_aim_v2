#include "point.hpp"

#include "utility/panic.hpp"
#include "utility/rclcpp/node.details.hpp"

#include <format>

#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

using namespace rmcs::util::visual;

using Marker = visualization_msgs::msg::Marker;

struct Points::Impl {
    static inline rclcpp::Clock rclcpp_clock { RCL_STEADY_TIME };

    Config config;

    Marker marker;
    std::shared_ptr<rclcpp::Publisher<Marker>> rclcpp_pub;

    explicit Impl(Config config) noexcept
        : config { std::move(config) } {
        initialize();
    }

    static auto create_rclcpp_publisher(const Config& config) noexcept
        -> std::shared_ptr<rclcpp::Publisher<Marker>> {
        const auto topic_name = config.rclcpp.get_pub_topic_prefix() + config.name;
        return config.rclcpp.details->make_pub<Marker>(topic_name, qos::debug);
    }

    auto initialize() noexcept -> void {
        if (!prefix::check_naming(config.name) || !prefix::check_naming(config.tf)) {
            util::panic(std::format(
                "Not a valid naming for points name or tf: {}", prefix::naming_standard));
        }

        marker.header.frame_id = config.tf;
        marker.ns              = config.name;
        marker.id              = config.id;
        marker.type            = Marker::POINTS;
        marker.action          = Marker::ADD;
        marker.lifetime        = rclcpp::Duration::from_seconds(0.1);

        marker.scale.x = config.size;
        marker.scale.y = config.size;
    }

    auto update(std::span<const Point> points) noexcept -> void {
        if (!rclcpp_pub) {
            rclcpp_pub = create_rclcpp_publisher(config);
        }

        marker.points.clear();
        marker.colors.clear();
        marker.points.reserve(points.size());
        marker.colors.reserve(points.size());

        for (const auto& point : points) {
            auto position = geometry_msgs::msg::Point { };
            position.x    = point.x;
            position.y    = point.y;
            position.z    = point.z;
            marker.points.emplace_back(position);

            auto color = std_msgs::msg::ColorRGBA { };
            color.r    = point.r;
            color.g    = point.g;
            color.b    = point.b;
            color.a    = point.a;
            marker.colors.emplace_back(color);
        }

        marker.header.stamp = rclcpp_clock.now();
        rclcpp_pub->publish(marker);
    }
};

auto Points::update(std::span<const Point> points) noexcept -> void { pimpl->update(points); }

Points::Points(const Config& config) noexcept
    : pimpl { std::make_unique<Impl>(config) } { }

Points::~Points() noexcept = default;

Points::Points(Points&&) noexcept                    = default;
auto Points::operator=(Points&&) noexcept -> Points& = default;
