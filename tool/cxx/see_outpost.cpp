#include "utility/math/outpost.hpp"
#include "utility/rclcpp/node.details.hpp"
#include "utility/robot/armor.hpp"
#include "utility/robot/constant.hpp"

#include <chrono>
#include <eigen3/Eigen/Geometry>
#include <format>
#include <numbers>
#include <rclcpp/utilities.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace rmcs;
using namespace rmcs::util;

auto main() -> int {
    rclcpp::init(0, nullptr);

    auto node = RclcppNode { "see_outpost" };
    node.set_pub_topic_prefix("/rmcs/auto_aim/outpost_test/");

    auto armor_pub = node.details->make_pub<visualization_msgs::msg::MarkerArray>(
        "/rmcs/auto_aim/outpost_test/armors", qos::debug);
    auto bar_pub = node.details->make_pub<visualization_msgs::msg::MarkerArray>(
        "/rmcs/auto_aim/outpost_test/bars", qos::debug);

    auto solution = NeighborBarSolution { };

    const Eigen::Vector3d center { 2.0, 0.0, 2.0 };
    auto start_time = std::chrono::steady_clock::now();

    while (rclcpp::ok()) {
        const auto stamp = node.details->rclcpp->now();
        const auto speed = kOutpostAngularSpeed * 0.1;
        double elapsed =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
        double angle = speed * elapsed;
        Eigen::Vector3d position { center.x() + kOutpostRadius * std::cos(angle),
            center.y() + kOutpostRadius * std::sin(angle), center.z() };

        const auto direction = position - center;
        const auto yaw_rotation =
            Eigen::AngleAxisd { std::atan2(direction.y(), direction.x()) + std::numbers::pi,
                Eigen::Vector3d::UnitZ() };
        const auto pitch_rotation =
            Eigen::AngleAxisd { kPredictedOutpostArmorPitch, Eigen::Vector3d::UnitY() };
        const auto armor = Armor3d {
            .genre       = DeviceId::OUTPOST,
            .color       = ArmorColor::BLUE,
            .id          = 0,
            .translation = Translation { position },
            .orientation = Orientation { Eigen::Quaterniond { yaw_rotation * pitch_rotation } },
        };

        auto armor_markers = visualization_msgs::msg::MarkerArray { };
        auto bar_markers   = visualization_msgs::msg::MarkerArray { };
        int marker_id      = 0;

        auto make_marker = [&](const std::string& ns, int id) {
            auto marker            = visualization_msgs::msg::Marker { };
            marker.header.frame_id = "camera_link";
            marker.header.stamp    = stamp;
            marker.ns              = ns;
            marker.id              = id;
            marker.action          = visualization_msgs::msg::Marker::ADD;
            marker.lifetime        = rclcpp::Duration::from_seconds(0.1);
            return marker;
        };

        auto make_point = [](const Point3d& p) {
            auto point = geometry_msgs::msg::Point { };
            p.copy_to(point);
            return point;
        };

        {
            auto marker    = make_marker("armor", 0);
            marker.type    = visualization_msgs::msg::Marker::CUBE;
            marker.scale.x = 0.003;
            marker.scale.y = 0.135;
            marker.scale.z = 0.056;
            marker.color.b = 1.0;
            marker.color.a = 1.0;
            armor.translation.copy_to(marker.pose.position);
            armor.orientation.copy_to(marker.pose.orientation);
            armor_markers.markers.emplace_back(std::move(marker));
        }

        const auto t = armor.translation.make<Eigen::Vector3d>();
        const auto q = armor.orientation.make<Eigen::Quaterniond>();

        const auto lpoint = Eigen::Vector3d { t + q * Eigen::Vector3d::UnitY() };
        const auto rpoint = Eigen::Vector3d { t + q * -Eigen::Vector3d::UnitY() };

        const auto pose_right = lpoint.norm() < rpoint.norm();
        const auto find_right = !pose_right;

        solution.input.source   = armor;
        solution.input.in_right = find_right;
        solution.solve();

        {
            auto marker    = make_marker("center", 0);
            marker.type    = visualization_msgs::msg::Marker::SPHERE;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.g = 1.0;
            marker.color.a = 1.0;
            solution.result.center.copy_to(marker.pose.position);
            armor_markers.markers.emplace_back(std::move(marker));
        }

        const auto publish_bar = [&](const Lightbar3d& bar, bool is_near) {
            auto marker = make_marker(
                std::format("{}_{}", find_right ? "right" : "left", is_near ? "near" : "away"),
                marker_id++);
            marker.type    = visualization_msgs::msg::Marker::LINE_LIST;
            marker.scale.x = 0.01;
            marker.color.r = is_near ? 1.0 : 0.0;
            marker.color.b = is_near ? 0.0 : 1.0;
            marker.color.a = 1.0;
            marker.points.emplace_back(make_point(bar.upper));
            marker.points.emplace_back(make_point(bar.lower));
            bar_markers.markers.emplace_back(std::move(marker));
        };

        publish_bar(solution.result.upper_near, true);
        publish_bar(solution.result.upper_away, false);
        publish_bar(solution.result.lower_near, true);
        publish_bar(solution.result.lower_away, false);

        armor_pub->publish(armor_markers);
        bar_pub->publish(bar_markers);

        rclcpp::sleep_for(std::chrono::milliseconds { 10 });
    }

    return 0;
}
