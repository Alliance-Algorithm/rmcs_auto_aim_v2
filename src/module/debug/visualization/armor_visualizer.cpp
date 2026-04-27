#include "armor_visualizer.hpp"

#include "utility/panic.hpp"
#include "utility/rclcpp/node.details.hpp"
#include "utility/robot/armor.hpp"

#include <format>
#include <unordered_set>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace rmcs::debug;
using Marker      = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;

namespace {

auto make_unique_marker_id(rmcs::DeviceId device, int armor_index) -> int {
    constexpr auto kArmorIndexBitWidth = 16;
    constexpr auto kArmorIndexLimit    = 1 << kArmorIndexBitWidth;

    if (armor_index < 0 || armor_index >= kArmorIndexLimit) {
        rmcs::util::panic(std::format("Armor marker index out of range: {}", armor_index));
    }

    auto const device_index = static_cast<int>(rmcs::to_index(device));
    return (device_index << kArmorIndexBitWidth) | armor_index;
}

auto set_marker_scale(Marker& marker, rmcs::DeviceId device, bool is_arrow) -> void {
    if (is_arrow) {
        marker.scale.x = 0.2;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;
        return;
    }

    if (rmcs::DeviceIds::kSmallArmor().contains(device)) {
        marker.scale.x = 0.003;
        marker.scale.y = 0.140;
        marker.scale.z = 0.125;
    } else if (rmcs::DeviceIds::kLargeArmor().contains(device)) {
        marker.scale.x = 0.003;
        marker.scale.y = 0.235;
        marker.scale.z = 0.127;
    }
}

auto set_marker_color(Marker& marker, rmcs::CampColor camp) -> void {
    if (camp == rmcs::CampColor::RED) {
        marker.color.r = 1.;
        marker.color.g = 0.;
        marker.color.b = 0.;
        marker.color.a = 1.;
    } else if (camp == rmcs::CampColor::BLUE) {
        marker.color.r = 0.;
        marker.color.g = 0.;
        marker.color.b = 1.;
        marker.color.a = 1.;
    } else {
        marker.color.r = 1.;
        marker.color.g = 0.;
        marker.color.b = 1.;
        marker.color.a = 1.;
    }
}

auto make_marker(std::string_view frame_id, std::string_view ns, int id, int type, int action,
    rmcs::DeviceId device, rmcs::CampColor camp, const rmcs::Armor3D* armor,
    const rclcpp::Time& stamp) -> Marker {
    auto marker            = Marker { };
    marker.header.frame_id = frame_id;
    marker.header.stamp    = stamp;
    marker.ns              = std::string { ns };
    marker.id              = id;
    marker.type            = type;
    marker.action          = action;
    marker.lifetime        = rclcpp::Duration::from_seconds(0.1);

    if (type == Marker::ARROW) {
        set_marker_scale(marker, device, true);
    } else {
        set_marker_scale(marker, device, false);
    }

    set_marker_color(marker, camp);

    if (armor) {
        armor->translation.copy_to(marker.pose.position);
        armor->orientation.copy_to(marker.pose.orientation);
    }

    return marker;
}

} // namespace

struct ArmorVisualizer::Impl final {
    auto initialize(util::RclcppNode& visual_node) noexcept -> void {
        node = std::ref(visual_node);
    }

    auto visualize(std::span<Armor3D const> armors, std::string const& name,
        std::string const& link_name) -> bool {
        if (!node.has_value()) {
            return false;
        }

        if (!rmcs::util::prefix::check_naming(name)
            || !rmcs::util::prefix::check_naming(link_name)) {
            util::panic(std::format("Not a valid naming for armor name or tf: {}",
                rmcs::util::prefix::naming_standard));
        }

        auto const topic_name = node.value().get().get_pub_topic_prefix() + name;
        if (!rclcpp_pub || published_topic != topic_name) {
            rclcpp_pub = node.value().get().details->make_pub<MarkerArray>(
                topic_name, rmcs::util::qos::debug);
            published_topic = topic_name;
            previous_ids.clear();
        }

        auto visual_marker      = MarkerArray { };
        const auto current_time = rclcpp_clock.now();
        auto current_ids        = std::unordered_set<int> { };
        auto const arrow_name   = std::format("{}_arrow", name);
        current_ids.reserve(armors.size());

        for (auto const& armor : armors) {
            auto const camp      = armor_color2camp_color(armor.color);
            auto const marker_id = make_unique_marker_id(armor.genre, armor.id);
            current_ids.emplace(marker_id);

            visual_marker.markers.emplace_back(make_marker(link_name, name, marker_id, Marker::CUBE,
                Marker::ADD, armor.genre, camp, &armor, current_time));
            visual_marker.markers.emplace_back(make_marker(link_name, arrow_name, marker_id,
                Marker::ARROW, Marker::ADD, armor.genre, camp, &armor, current_time));
        }

        for (auto const id : previous_ids) {
            if (current_ids.contains(id)) {
                continue;
            }

            visual_marker.markers.emplace_back(make_marker(link_name, name, id, Marker::CUBE,
                Marker::DELETE, rmcs::DeviceId { }, rmcs::CampColor { }, nullptr, current_time));
            visual_marker.markers.emplace_back(make_marker(link_name, arrow_name, id, Marker::ARROW,
                Marker::DELETE, rmcs::DeviceId { }, rmcs::CampColor { }, nullptr, current_time));
        }

        previous_ids = std::move(current_ids);
        rclcpp_pub->publish(visual_marker);
        return true;
    }

    static inline rclcpp::Clock rclcpp_clock { RCL_STEADY_TIME };

    std::optional<std::reference_wrapper<util::RclcppNode>> node;
    std::shared_ptr<rclcpp::Publisher<MarkerArray>> rclcpp_pub;
    std::string published_topic;
    std::unordered_set<int> previous_ids;
};

auto ArmorVisualizer::initialize(util::RclcppNode& visual_node) noexcept -> void {
    return pimpl->initialize(visual_node);
}

auto ArmorVisualizer::visualize(std::span<Armor3D const> armors, std::string const& name,
    std::string const& link_name) -> bool {
    return pimpl->visualize(armors, name, link_name);
}

ArmorVisualizer::ArmorVisualizer() noexcept
    : pimpl { std::make_unique<Impl>() } { };
ArmorVisualizer::~ArmorVisualizer() noexcept = default;
