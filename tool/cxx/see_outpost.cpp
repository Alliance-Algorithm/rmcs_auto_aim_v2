#include "utility/math/outpost.hpp"
#include "utility/rclcpp/node.details.hpp"
#include "utility/robot/armor.hpp"
#include "utility/robot/constant.hpp"

#include <eigen3/Eigen/Geometry>
#include <rclcpp/utilities.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace rmcs;
using namespace rmcs::util;

// 辅助函数：创建前哨站装甲板
auto make_outpost_armor(int id, const Eigen::Vector3d& position) -> Armor3D {
    const auto yaw_angle    = std::atan2(position.y(), position.x());
    const auto yaw_rotation = Eigen::AngleAxisd { yaw_angle, Eigen::Vector3d::UnitZ() };
    const auto pitch_rotation =
        Eigen::AngleAxisd { kPredictedOutpostArmorPitch, Eigen::Vector3d::UnitY() };
    const auto orientation = Eigen::Quaterniond { yaw_rotation * pitch_rotation };

    return Armor3D {
        .genre       = DeviceId::OUTPOST,
        .color       = ArmorColor::BLUE,
        .id          = id,
        .translation = Translation { position },
        .orientation = Orientation { orientation },
    };
}

// 辅助函数：创建 Point
auto make_point(const Point3d& p) -> geometry_msgs::msg::Point {
    auto point = geometry_msgs::msg::Point { };
    p.copy_to(point);
    return point;
}

// 辅助函数：创建 Marker 基础信息
auto make_marker_base(const std::string& ns, int id, const rclcpp::Time& stamp)
    -> visualization_msgs::msg::Marker {
    auto marker            = visualization_msgs::msg::Marker { };
    marker.header.frame_id = "camera_link";
    marker.header.stamp    = stamp;
    marker.ns              = ns;
    marker.id              = id;
    marker.action          = visualization_msgs::msg::Marker::ADD;
    marker.lifetime        = rclcpp::Duration::from_seconds(0.1);
    return marker;
}

auto main() -> int {
    rclcpp::init(0, nullptr);

    auto node = RclcppNode { "see_outpost" };
    node.set_pub_topic_prefix("/rmcs/auto_aim/outpost_test/");

    auto armor_pub = node.details->make_pub<visualization_msgs::msg::MarkerArray>(
        "/rmcs/auto_aim/outpost_test/armors", qos::debug);
    auto bar_pub = node.details->make_pub<visualization_msgs::msg::MarkerArray>(
        "/rmcs/auto_aim/outpost_test/bars", qos::debug);

    // 创建 3 个装甲板
    const auto armors = std::array {
        make_outpost_armor(0, { 2.0, +0.0, 2.0 }),
        make_outpost_armor(1, { 2.0, +2.0, 2.0 }),
        make_outpost_armor(2, { 2.0, -2.0, 2.0 }),
    };

    auto solution = NeighborBarSolution { };

    while (rclcpp::ok()) {
        const auto stamp = node.details->rclcpp->now();

        auto armor_markers = visualization_msgs::msg::MarkerArray { };
        auto bar_markers   = visualization_msgs::msg::MarkerArray { };
        int marker_id      = 0;

        for (size_t i = 0; i < armors.size(); ++i) {
            const auto& armor = armors[i];

            // 发布装甲板位置
            {
                auto marker    = make_marker_base("armor", static_cast<int>(i), stamp);
                marker.type    = visualization_msgs::msg::Marker::CUBE;
                marker.scale.x = 0.003;
                marker.scale.y = 0.135;
                marker.scale.z = 0.056;
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 1.0;
                marker.color.a = 1.0;

                armor.translation.copy_to(marker.pose.position);
                armor.orientation.copy_to(marker.pose.orientation);
                armor_markers.markers.emplace_back(std::move(marker));
            }

            // 计算旋转中心：先补偿掉装甲板倾角，再反推半径
            const auto armor_toward = Eigen::Vector3d { armor.orientation.make<Eigen::Quaterniond>()
                * Eigen::Vector3d::UnitX() };
            const auto horizon_toward = Eigen::Vector3d {
                Eigen::AngleAxisd { -kPredictedOutpostArmorPitch, Eigen::Vector3d::UnitY() }
                * armor_toward
            };
            const auto outpost_center = Eigen::Vector3d { armor.translation.make<Eigen::Vector3d>()
                + horizon_toward * kOutpostRadius };

            // 发布旋转中心
            {
                auto marker    = make_marker_base("center", static_cast<int>(i), stamp);
                marker.type    = visualization_msgs::msg::Marker::SPHERE;
                marker.scale.x = 0.1;
                marker.scale.y = 0.1;
                marker.scale.z = 0.1;
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marker.color.a = 1.0;

                marker.pose.position.x = outpost_center.x();
                marker.pose.position.y = outpost_center.y();
                marker.pose.position.z = outpost_center.z();
                armor_markers.markers.emplace_back(std::move(marker));
            }

            // 发布灯条（左右两侧）
            for (auto in_right : { false, true }) {
                solution.input.source   = armor;
                solution.input.in_right = in_right;
                solution.solve();

                const auto* side = in_right ? "right" : "left";

                const auto publish_bar = [&](const NeighborBarSolution::Result::Bar& bar,
                                             bool is_near) {
                    auto marker = make_marker_base(
                        std::string { side } + (is_near ? "_near" : "_away"), marker_id++, stamp);
                    marker.type    = visualization_msgs::msg::Marker::LINE_LIST;
                    marker.scale.x = 0.01;

                    // near 红色，away 蓝色
                    marker.color.r = is_near ? 1.0 : 0.0;
                    marker.color.g = 0.0;
                    marker.color.b = is_near ? 0.0 : 1.0;
                    marker.color.a = 1.0;

                    marker.points.emplace_back(make_point(bar.first));
                    marker.points.emplace_back(make_point(bar.second));
                    bar_markers.markers.emplace_back(std::move(marker));
                };

                publish_bar(solution.result.upper_near, true);
                publish_bar(solution.result.upper_away, false);
                publish_bar(solution.result.lower_near, true);
                publish_bar(solution.result.lower_away, false);
            }
        }

        armor_pub->publish(armor_markers);
        bar_pub->publish(bar_markers);

        rclcpp::sleep_for(std::chrono::milliseconds { 10 });
    }

    return 0;
}
