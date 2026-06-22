#include "module/predictor/model/outpost.hpp"
#include "utility/math/angle.hpp"
#include "utility/math/linear.hpp"
#include "utility/rclcpp/node.details.hpp"
#include "utility/robot/armor.hpp"
#include "utility/robot/constant.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <format>
#include <iostream>
#include <numbers>
#include <random>
#include <tuple>

#include <eigen3/Eigen/Geometry>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace rmcs;
using namespace rmcs::util;

auto main() -> int {
    rclcpp::init(0, nullptr);

    auto node = RclcppNode { "test_outpost_ekf" };
    node.set_pub_topic_prefix("/rmcs/auto_aim/outpost_test/");

    auto marker_publisher = node.details->make_pub<visualization_msgs::msg::MarkerArray>(
        "/rmcs/auto_aim/outpost_test/ekf", qos::debug);

    const auto kTrueCenter       = Eigen::Vector3d { 2.0, 0.0, 2.0 };
    constexpr auto kAngularSpeed = kOutpostAngularSpeed; // +0.8 pi, CCW
    constexpr auto kRadius       = kOutpostRadius;
    constexpr auto kPitch        = kPredictedOutpostArmorPitch;
    constexpr auto kDt           = 0.01;

    static constexpr auto kOffsetTable = std::array {
        std::tuple { 0 * std::numbers::pi * 2 / 3, -0 * kOutpostArmorHeightStep }, // 高
        std::tuple { 1 * std::numbers::pi * 2 / 3, -1 * kOutpostArmorHeightStep }, // 中
        std::tuple { 2 * std::numbers::pi * 2 / 3, -2 * kOutpostArmorHeightStep }, // 低
    };

    constexpr auto kNoiseSigma               = 0.03;
    constexpr auto kOutlierProbability       = 0.0;
    constexpr auto kOutlierSigma             = 1.0;
    constexpr auto kOrientationSigma         = 0.05;
    constexpr auto kObservationCloudCapacity = 50;

    auto noise_engine         = std::mt19937 { 42 };
    auto noise_distribution   = std::normal_distribution<double> { 0.0, kNoiseSigma };
    auto outlier_distribution = std::normal_distribution<double> { 0.0, kOutlierSigma };
    auto orientation_noise_distribution =
        std::normal_distribution<double> { 0.0, kOrientationSigma };
    auto uniform_distribution = std::uniform_real_distribution<double> { 0.0, 1.0 };

    auto model             = std::unique_ptr<OutpostModel> { };
    auto elapsed           = 0.0;
    auto frame_index       = 0;
    auto last_observed_id  = int { -1 };
    auto observation_cloud = std::deque<Eigen::Vector3d> { };

    const auto kStartTime = std::chrono::steady_clock::now();

    while (rclcpp::ok()) {
        elapsed += kDt;
        const auto stamp = node.details->rclcpp->now();

        const auto angle = kAngularSpeed * elapsed;

        auto true_positions    = std::array<Eigen::Vector3d, kOffsetTable.size()> { };
        auto true_orientations = std::array<Eigen::Quaterniond, kOffsetTable.size()> { };
        for (std::size_t index = 0; index < kOffsetTable.size(); ++index) {
            const auto [yaw_offset, height_offset] = kOffsetTable[index];
            const auto armor_yaw                   = util::normalize_angle(angle + yaw_offset);

            true_positions[index] = Eigen::Vector3d {
                kTrueCenter.x() - kRadius * std::cos(armor_yaw),
                kTrueCenter.y() - kRadius * std::sin(armor_yaw),
                kTrueCenter.z() + height_offset,
            };
            true_orientations[index] =
                Eigen::Quaterniond { Eigen::AngleAxisd { armor_yaw, Eigen::Vector3d::UnitZ() }
                    * Eigen::AngleAxisd { kPitch, Eigen::Vector3d::UnitY() } };
        }

        auto observed_id = std::size_t { 0 };
        for (std::size_t index = 1; index < true_positions.size(); ++index) {
            if (true_positions[index].x() >= true_positions[observed_id].x()) continue;
            observed_id = index;
        }

        const auto plate_switched =
            last_observed_id >= 0 && observed_id != static_cast<std::size_t>(last_observed_id);
        last_observed_id = static_cast<int>(observed_id);

        const auto true_position    = true_positions[observed_id];
        const auto true_orientation = true_orientations[observed_id];

        auto distance_direction = Eigen::Vector3d { true_position.normalized() };
        auto noisy_position     = Eigen::Vector3d { true_position
            + distance_direction * noise_distribution(noise_engine) };

        auto is_outlier = uniform_distribution(noise_engine) < kOutlierProbability;
        if (is_outlier) {
            noisy_position += distance_direction * outlier_distribution(noise_engine);
        }

        observation_cloud.push_back(noisy_position);
        if (observation_cloud.size() > kObservationCloudCapacity) {
            observation_cloud.pop_front();
        }

        const auto orientation_perturbation =
            Eigen::AngleAxisd { orientation_noise_distribution(noise_engine),
                Eigen::Vector3d::UnitZ() };
        const auto noisy_orientation =
            Eigen::Quaterniond { orientation_perturbation * true_orientation };

        auto noisy_armor = Armor3d {
            .genre       = DeviceId::OUTPOST,
            .color       = ArmorColor::BLUE,
            .id          = 0,
            .translation = Translation { noisy_position },
            .orientation = Orientation { noisy_orientation },
        };

        if (!model) {
            model = std::make_unique<OutpostModel>(noisy_armor);
        } else {
            model->predict(kDt);
            model->correct(noisy_armor);
        }

        const auto estimated_state  = model->state();
        const auto estimated_center = Eigen::Vector3d {
            estimated_state.x,
            estimated_state.y,
            estimated_state.z,
        };
        const auto estimated_armors = model->full();

        auto marker_array = visualization_msgs::msg::MarkerArray { };
        auto marker_index = 0;

        auto make_marker = [&](const std::string& namespace_name) {
            auto marker            = visualization_msgs::msg::Marker { };
            marker.header.frame_id = "camera_link";
            marker.header.stamp    = stamp;
            marker.ns              = namespace_name;
            marker.id              = marker_index++;
            marker.action          = visualization_msgs::msg::Marker::ADD;
            marker.lifetime        = rclcpp::Duration::from_seconds(0.1);
            return marker;
        };

        auto make_point = [](const Eigen::Vector3d& position) {
            auto point = geometry_msgs::msg::Point { };
            point.x    = position.x();
            point.y    = position.y();
            point.z    = position.z();
            return point;
        };

        auto set_pose = [](visualization_msgs::msg::Marker& marker, const Eigen::Vector3d& position,
                            const Eigen::Quaterniond& orientation) {
            marker.pose.position.x    = position.x();
            marker.pose.position.y    = position.y();
            marker.pose.position.z    = position.z();
            marker.pose.orientation.w = orientation.w();
            marker.pose.orientation.x = orientation.x();
            marker.pose.orientation.y = orientation.y();
            marker.pose.orientation.z = orientation.z();
        };

        for (std::size_t index = 0; index < true_positions.size(); ++index) {
            auto marker    = make_marker("true_armor");
            marker.type    = visualization_msgs::msg::Marker::CUBE;
            marker.scale.x = 0.003;
            marker.scale.y = 0.135;
            marker.scale.z = 0.056;
            marker.color.g = 1.0;
            marker.color.b = index == observed_id ? 0.0 : 1.0;
            marker.color.a = index == observed_id ? 1.0 : 0.25;
            set_pose(marker, true_positions[index], true_orientations[index]);
            marker_array.markers.emplace_back(std::move(marker));
        }

        {
            auto marker    = make_marker("observed_armor");
            marker.type    = visualization_msgs::msg::Marker::CUBE;
            marker.scale.x = 0.003;
            marker.scale.y = 0.135;
            marker.scale.z = 0.056;
            marker.color.r = 1.0;
            marker.color.b = 1.0;
            marker.color.a = 0.7;
            set_pose(marker, noisy_position, noisy_orientation);
            marker_array.markers.emplace_back(std::move(marker));
        }

        {
            auto marker    = make_marker("noisy_observation");
            marker.type    = visualization_msgs::msg::Marker::POINTS;
            marker.scale.x = 0.02;
            marker.scale.y = 0.02;
            marker.color.r = 1.0;
            marker.color.a = 0.6;
            for (const auto& position : observation_cloud) {
                marker.points.emplace_back(make_point(position));
            }
            marker_array.markers.emplace_back(std::move(marker));
        }

        {
            auto marker    = make_marker("estimated_center");
            marker.type    = visualization_msgs::msg::Marker::SPHERE;
            marker.scale.x = 0.06;
            marker.scale.y = 0.06;
            marker.scale.z = 0.06;
            marker.color.b = 1.0;
            marker.color.a = 1.0;
            set_pose(marker, estimated_center, Eigen::Quaterniond::Identity());
            marker_array.markers.emplace_back(std::move(marker));
        }

        for (std::size_t index = 0; index < estimated_armors.size(); ++index) {
            const auto& armor      = estimated_armors[index];
            const auto position    = armor.translation.make<Eigen::Vector3d>();
            const auto orientation = armor.orientation.make<Eigen::Quaterniond>();

            auto marker    = make_marker("estimated_armor");
            marker.type    = visualization_msgs::msg::Marker::CUBE;
            marker.scale.x = 0.003;
            marker.scale.y = 0.135;
            marker.scale.z = 0.056;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.a = 1.0;
            set_pose(marker, position, orientation);
            marker_array.markers.emplace_back(std::move(marker));
        }

        {
            auto marker    = make_marker("estimated_circle");
            marker.type    = visualization_msgs::msg::Marker::LINE_STRIP;
            marker.scale.x = 0.005;
            marker.color.b = 1.0;
            marker.color.a = 0.5;
            for (int step = 0; step <= 36; ++step) {
                const auto phi = 2.0 * std::numbers::pi * step / 36.0;
                marker.points.emplace_back(make_point(Eigen::Vector3d {
                    estimated_center.x() + kRadius * std::cos(phi),
                    estimated_center.y() + kRadius * std::sin(phi),
                    estimated_center.z(),
                }));
            }
            marker_array.markers.emplace_back(std::move(marker));
        }

        marker_publisher->publish(marker_array);

        if (plate_switched || frame_index % 10 == 0) {
            const auto reference_center = Eigen::Vector3d {
                kTrueCenter.x(),
                kTrueCenter.y(),
                kTrueCenter.z() + std::get<1>(kOffsetTable[0]),
            };
            const auto center_error = (estimated_center - reference_center).norm();

            // state.rotation_angle 约定为 center→armor 方向，true_yaw 需相应转换
            // test 的 angle 是 armor→center 方向，加 π 得到 center→armor
            const auto true_yaw =
                util::normalize_angle(angle + std::get<0>(kOffsetTable[0]) + std::numbers::pi);
            const auto angle_error =
                std::abs(util::normalize_angle(estimated_state.rotation_angle - true_yaw));
            std::cout << std::format("elapsed={:6.2f}s center_err={:.4f}m omega_est={:.4f} "
                                     "omega_true={:.4f} angle_err={:.4f}rad obs={}{}{}\n",
                elapsed, center_error, estimated_state.rotation_speed, kAngularSpeed, angle_error,
                observed_id, plate_switched ? " [PLATE_SWITCH]" : "",
                is_outlier ? " [OUTLIER]" : "");
        }

        ++frame_index;
        rclcpp::sleep_for(std::chrono::milliseconds { 10 });
    }

    return 0;
}
