#include "lightbar.hpp"

#include "utility/panic.hpp"
#include "utility/rclcpp/node.details.hpp"

#include <format>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace rmcs;
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

    auto set_color(float r, float g, float b, float a) noexcept -> void {
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = a;
    }

    auto update() noexcept -> void {
        if (!rclcpp_pub) {
            rclcpp_pub = create_rclcpp_publisher(config);
        }

        marker.header.stamp = rclcpp_clock.now();
        rclcpp_pub->publish(marker);
    }
};

auto LightBar::set_point(const Point3d& top, const Point3d& bottom) noexcept -> void {
    pimpl->set(top, bottom);
}

auto LightBar::set_color(float r, float g, float b, float a) noexcept -> void {
    pimpl->set_color(r, g, b, a);
}

auto LightBar::update() noexcept -> void { pimpl->update(); }

LightBar::LightBar(const Config& config) noexcept
    : pimpl { std::make_unique<Impl>(config) } { }

LightBar::~LightBar() noexcept = default;

using MarkerArray = visualization_msgs::msg::MarkerArray;

struct Lightbars::Impl {
    static inline rclcpp::Clock rclcpp_clock { RCL_STEADY_TIME };

    Config config;

    std::shared_ptr<rclcpp::Publisher<MarkerArray>> rclcpp_pub;
    std::size_t previous_size = 0;

    explicit Impl(Config config) noexcept
        : config { std::move(config) } {
        if (!prefix::check_naming(this->config.name) || !prefix::check_naming(this->config.tf)) {
            util::panic(std::format(
                "Not a valid naming for lightbars name or tf: {}", prefix::naming_standard));
        }
    }

    static auto create_rclcpp_publisher(const Config& config) noexcept
        -> std::shared_ptr<rclcpp::Publisher<MarkerArray>> {
        const auto topic_name = config.rclcpp.get_pub_topic_prefix() + config.name;
        return config.rclcpp.details->make_pub<MarkerArray>(topic_name, qos::debug);
    }

    auto make_marker(int id, const Lightbar3d& bar, const rclcpp::Time& stamp) const noexcept
        -> Marker {
        auto marker            = Marker { };
        marker.header.frame_id = config.tf;
        marker.header.stamp    = stamp;
        marker.ns              = config.name;
        marker.id              = id;
        marker.type            = Marker::LINE_LIST;
        marker.action          = Marker::ADD;
        marker.lifetime        = rclcpp::Duration::from_seconds(0.5);

        marker.scale.x = 0.01;

        ArmorVisualColor { bar.color }.to(marker.color);

        marker.points.resize(2);
        marker.points[0].x = bar.upper.x;
        marker.points[0].y = bar.upper.y;
        marker.points[0].z = bar.upper.z;
        marker.points[1].x = bar.lower.x;
        marker.points[1].y = bar.lower.y;
        marker.points[1].z = bar.lower.z;

        return marker;
    }

    auto make_delete_marker(int id, const rclcpp::Time& stamp) const noexcept -> Marker {
        auto marker            = Marker { };
        marker.header.frame_id = config.tf;
        marker.header.stamp    = stamp;
        marker.ns              = config.name;
        marker.id              = id;
        marker.type            = Marker::LINE_LIST;
        marker.action          = Marker::DELETE;
        return marker;
    }

    auto update(std::span<const Lightbar3d> lightbars) noexcept -> void {
        if (!rclcpp_pub) {
            rclcpp_pub = create_rclcpp_publisher(config);
        }

        auto visual_marker = MarkerArray { };
        const auto stamp   = rclcpp_clock.now();

        for (int i = 0; i < static_cast<int>(lightbars.size()); ++i) {
            visual_marker.markers.emplace_back(make_marker(i, lightbars[i], stamp));
        }

        for (std::size_t i = lightbars.size(); i < previous_size; ++i) {
            visual_marker.markers.emplace_back(
                make_delete_marker(static_cast<int>(i), stamp));
        }

        previous_size = lightbars.size();
        rclcpp_pub->publish(visual_marker);
    }
};

auto Lightbars::update(std::span<const Lightbar3d> lightbars) noexcept -> void {
    pimpl->update(lightbars);
}

Lightbars::Lightbars(Config config) noexcept
    : pimpl { std::make_unique<Impl>(std::move(config)) } { }

Lightbars::~Lightbars() noexcept = default;

Lightbars::Lightbars(Lightbars&&) noexcept = default;

auto Lightbars::operator=(Lightbars&&) noexcept -> Lightbars& = default;
