#include "armor.hpp"
#include "utility/panic.hpp"
#include "utility/rclcpp/node.details.hpp"

#include <visualization_msgs/msg/marker_array.hpp>

using namespace rmcs::util::visual;

using Marker      = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;

struct Armor::Impl {
    static inline rclcpp::Clock rclcpp_clock { RCL_STEADY_TIME };

    Config config;

    Marker marker;
    Marker arrow_marker;
    std::shared_ptr<rclcpp::Publisher<MarkerArray>> rclcpp_pub;

    explicit Impl(Config config)
        : config(std::move(config)) {
        initialize();
    }

    static auto create_rclcpp_publisher(Config const& config) noexcept
        -> std::shared_ptr<rclcpp::Publisher<MarkerArray>> {
        const auto topic_name { config.rclcpp.get_pub_topic_prefix() + config.name };
        return config.rclcpp.details->make_pub<MarkerArray>(topic_name, qos::debug);
    }

    auto initialize() -> void {
        if (!prefix::check_naming(config.name) || !prefix::check_naming(config.tf)) {
            util::panic(std::format(
                "Not a valid naming for armor name or tf: {}", prefix::naming_standard));
        }

        marker.header.frame_id = config.tf;
        marker.ns              = config.name;
        marker.id              = config.id;
        marker.type            = Marker::CUBE;
        marker.action          = Marker::ADD;
        marker.lifetime        = rclcpp::Duration::from_seconds(0.1);

        // ref: "https://www.robomaster.com/zh-CN/products/components/detail/149"
        /*  */ if (DeviceIds::kSmallArmorDevices().contains(config.device)) {
            marker.scale.x = 0.003, marker.scale.y = 0.140, marker.scale.z = 0.125;
        } else if (DeviceIds::kLargeArmorDevices().contains(config.device)) {
            marker.scale.x = 0.003, marker.scale.y = 0.235, marker.scale.z = 0.127;
        } else {
            util::panic("Wrong device id for a visualized armor");
        };

        /*  */ if (config.camp == CampColor::RED) {
            marker.color.r = 1., marker.color.g = 0., marker.color.b = 0., marker.color.a = 1.;
        } else if (config.camp == CampColor::BLUE) {
            marker.color.r = 0., marker.color.g = 0., marker.color.b = 1., marker.color.a = 1.;
        } else {
            marker.color.r = 1., marker.color.g = 0., marker.color.b = 1., marker.color.a = 1.;
        }

        arrow_marker.header.frame_id = config.tf;
        arrow_marker.ns              = config.name + std::string("_arrow");
        arrow_marker.id              = config.id;
        arrow_marker.type            = Marker::ARROW;
        arrow_marker.action          = Marker::ADD;
        arrow_marker.lifetime        = rclcpp::Duration::from_seconds(0.1);

        arrow_marker.scale.x = 0.2;
        arrow_marker.scale.y = 0.01;
        arrow_marker.scale.z = 0.01;

        /*  */ if (config.camp == CampColor::RED) {
            arrow_marker.color.r = 1., arrow_marker.color.g = 0., arrow_marker.color.b = 0.,
            arrow_marker.color.a = 1.;
        } else if (config.camp == CampColor::BLUE) {
            arrow_marker.color.r = 0., arrow_marker.color.g = 0., arrow_marker.color.b = 1.,
            arrow_marker.color.a = 1.;
        } else {
            arrow_marker.color.r = 1., arrow_marker.color.g = 0., arrow_marker.color.b = 1.,
            arrow_marker.color.a = 1.;
        }
    }

    auto update() noexcept -> void {
        if (!rclcpp_pub) {
            rclcpp_pub = create_rclcpp_publisher(config);
        }

        MarkerArray visual_marker;
        const auto current_stamp  = rclcpp_clock.now();
        marker.header.stamp       = current_stamp;
        arrow_marker.header.stamp = current_stamp;

        arrow_marker.pose = marker.pose;
        visual_marker.markers.emplace_back(marker);
        visual_marker.markers.emplace_back(arrow_marker);

        rclcpp_pub->publish(visual_marker);
    }

    auto move(const Translation& t, const Orientation& q) noexcept {
        t.copy_to(marker.pose.position);
        q.copy_to(marker.pose.orientation);
    }
};

auto Armor::update() noexcept -> void { pimpl->update(); }

auto Armor::impl_move(const Translation& t, const Orientation& q) noexcept -> void {
    pimpl->move(t, q);
}

Armor::Armor(const Config& config) noexcept
    : pimpl { std::make_unique<Impl>(config) } { }

Armor::~Armor() noexcept = default;
