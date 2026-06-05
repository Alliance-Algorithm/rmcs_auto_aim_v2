#include "armor.hpp"
#include "utility/panic.hpp"
#include "utility/rclcpp/node.details.hpp"
#include "utility/robot/armor.hpp"

#include <format>
#include <unordered_set>

#include <visualization_msgs/msg/marker_array.hpp>

using namespace rmcs;
using namespace rmcs::util::visual;

using Marker      = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;

namespace {

auto make_unique_marker_id(DeviceId device, int armor_index) -> int {
    constexpr auto kArmorIndexBitWidth = 16;
    constexpr auto kArmorIndexLimit    = 1 << kArmorIndexBitWidth;

    if (armor_index < 0 || armor_index >= kArmorIndexLimit) {
        util::panic(std::format("Armor marker index out of range: {}", armor_index));
    }

    const auto device_index = static_cast<int>(to_index(device));
    return (device_index << kArmorIndexBitWidth) | armor_index;
}

auto make_armor_marker(const std::string& frame_id, const std::string& ns, int id,
    const Armor3d& armor, const rclcpp::Time& stamp) -> Marker {
    auto marker            = Marker { };
    marker.header.frame_id = frame_id;
    marker.header.stamp    = stamp;
    marker.ns              = ns;
    marker.id              = id;
    marker.type            = Marker::CUBE;
    marker.action          = Marker::ADD;
    marker.lifetime        = rclcpp::Duration::from_seconds(0.1);

    ArmorVisualScale { armor.genre }.to(marker.scale);
    ArmorVisualColor { armor_color2camp_color(armor.color) }.to(marker.color);
    armor.translation.copy_to(marker.pose.position);
    armor.orientation.copy_to(marker.pose.orientation);

    return marker;
}

auto make_arrow_marker(const std::string& frame_id, const std::string& ns, int id,
    const Armor3d& armor, const rclcpp::Time& stamp) -> Marker {
    auto marker            = Marker { };
    marker.header.frame_id = frame_id;
    marker.header.stamp    = stamp;
    marker.ns              = ns;
    marker.id              = id;
    marker.type            = Marker::ARROW;
    marker.action          = Marker::ADD;
    marker.lifetime        = rclcpp::Duration::from_seconds(1);

    marker.scale.x = 0.2;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;

    ArmorVisualColor { armor_color2camp_color(armor.color) }.to(marker.color);
    armor.translation.copy_to(marker.pose.position);
    armor.orientation.copy_to(marker.pose.orientation);

    return marker;
}

auto make_delete_marker(const std::string& frame_id, const std::string& ns, int id, int type,
    const rclcpp::Time& stamp) -> Marker {
    auto marker            = Marker { };
    marker.header.frame_id = frame_id;
    marker.header.stamp    = stamp;
    marker.ns              = ns;
    marker.id              = id;
    marker.type            = type;
    marker.action          = Marker::DELETE;
    return marker;
}

} // namespace

struct Armor::Impl {
    static inline rclcpp::Clock rclcpp_clock { RCL_STEADY_TIME };

    Config config;

    Marker marker;
    Marker arrow_marker;
    std::shared_ptr<rclcpp::Publisher<MarkerArray>> rclcpp_pub;

    explicit Impl(Config config)
        : config(std::move(config)) {
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

        ArmorVisualScale { config.device }.to(marker.scale);
        ArmorVisualColor { config.camp }.to(marker.color);

        arrow_marker.header.frame_id = config.tf;
        arrow_marker.ns              = config.name + std::string("_arrow");
        arrow_marker.id              = config.id;
        arrow_marker.type            = Marker::ARROW;
        arrow_marker.action          = Marker::ADD;
        arrow_marker.lifetime        = rclcpp::Duration::from_seconds(0.1);

        arrow_marker.scale.x = 0.2;
        arrow_marker.scale.y = 0.01;
        arrow_marker.scale.z = 0.01;

        ArmorVisualColor { config.camp }.to(arrow_marker.color);
    }

    static auto create_rclcpp_publisher(Config const& config) noexcept
        -> std::shared_ptr<rclcpp::Publisher<MarkerArray>> {
        const auto topic_name { config.rclcpp.get_pub_topic_prefix() + config.name };
        return config.rclcpp.details->make_pub<MarkerArray>(topic_name, qos::debug);
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

    auto set_camp(CampColor camp) noexcept {
        ArmorVisualColor { camp }.to(marker.color);
        ArmorVisualColor { camp }.to(arrow_marker.color);
    }
};

auto Armor::update() noexcept -> void { pimpl->update(); }

auto Armor::set_camp(CampColor camp) noexcept -> void { pimpl->set_camp(camp); }

auto Armor::impl_move(const Translation& t, const Orientation& q) noexcept -> void {
    pimpl->move(t, q);
}

Armor::Armor(const Config& config) noexcept
    : pimpl { std::make_unique<Impl>(config) } { }

Armor::~Armor() noexcept = default;

struct Armors::Impl {
    static inline rclcpp::Clock rclcpp_clock { RCL_STEADY_TIME };

    Config config;

    std::shared_ptr<rclcpp::Publisher<MarkerArray>> rclcpp_pub;
    std::unordered_set<int> previous_ids;

    explicit Impl(Config config) noexcept
        : config { std::move(config) } {
        if (!prefix::check_naming(this->config.name) || !prefix::check_naming(this->config.tf)) {
            util::panic(std::format(
                "Not a valid naming for armors name or tf: {}", prefix::naming_standard));
        }
    }

    static auto create_rclcpp_publisher(const Config& config) noexcept
        -> std::shared_ptr<rclcpp::Publisher<MarkerArray>> {
        const auto topic_name = config.rclcpp.get_pub_topic_prefix() + config.name;
        return config.rclcpp.details->make_pub<MarkerArray>(topic_name, qos::debug);
    }

    auto update(std::span<const Armor3d> armors) noexcept -> void {
        if (!rclcpp_pub) {
            rclcpp_pub = create_rclcpp_publisher(config);
        }

        auto visual_marker  = MarkerArray { };
        const auto stamp    = rclcpp_clock.now();
        auto current_ids    = std::unordered_set<int> { };
        const auto arrow_ns = config.name + std::string { "_arrow" };
        current_ids.reserve(armors.size());

        for (const auto& armor : armors) {
            const auto marker_id = make_unique_marker_id(armor.genre, armor.id);
            current_ids.emplace(marker_id);

            visual_marker.markers.emplace_back(
                make_armor_marker(config.tf, config.name, marker_id, armor, stamp));
            visual_marker.markers.emplace_back(
                make_arrow_marker(config.tf, arrow_ns, marker_id, armor, stamp));
        }

        for (const auto id : previous_ids) {
            if (current_ids.contains(id)) continue;

            visual_marker.markers.emplace_back(
                make_delete_marker(config.tf, config.name, id, Marker::CUBE, stamp));
            visual_marker.markers.emplace_back(
                make_delete_marker(config.tf, arrow_ns, id, Marker::ARROW, stamp));
        }

        previous_ids = std::move(current_ids);
        rclcpp_pub->publish(visual_marker);
    }
};

auto Armors::update(std::span<const Armor3d> armors) noexcept -> void { pimpl->update(armors); }

Armors::Armors(Config config) noexcept
    : pimpl { std::make_unique<Impl>(std::move(config)) } { }

Armors::~Armors() noexcept = default;

Armors::Armors(Armors&&) noexcept = default;

auto Armors::operator=(Armors&&) noexcept -> Armors& = default;
