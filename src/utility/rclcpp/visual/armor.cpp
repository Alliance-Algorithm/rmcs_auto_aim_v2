#include "armor.hpp"
#include "utility/panic.hpp"
#include "utility/rclcpp/node.details.hpp"
#include <rclcpp/publisher.hpp>
#include <visualization_msgs/msg/marker.hpp>

using namespace rmcs::util::visual;

using Marker = visualization_msgs::msg::Marker;

struct Armor::Impl {
    static inline rclcpp::Clock rclcpp_clock { RCL_SYSTEM_TIME };

    Marker marker;
    std::shared_ptr<rclcpp::Publisher<Marker>> rclcpp_pub;

    explicit Impl(const Config& config) {
        auto& rclcpp  = config.rclcpp;
        auto& details = config.rclcpp.details;

        if (!prefix::check_naming(config.id) || !prefix::check_naming(config.tf)) {
            util::panic(
                std::format("Not a valid naming for armor id or tf: {}", prefix::naming_standard));
        }

        const auto topic_name { rclcpp.get_pub_topic_prefix() + config.id };
        rclcpp_pub = details->make_pub<Marker>(topic_name, qos::debug);

        marker.header.frame_id = config.tf;

        marker.ns     = config.id;
        marker.id     = 0;
        marker.type   = Marker::CUBE;
        marker.action = Marker::ADD;

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
            util::panic("Please specify a valid armor color");
        }
    }

    auto update() noexcept -> void {
        marker.header.stamp = rclcpp_clock.now();
        rclcpp_pub->publish(marker);
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
