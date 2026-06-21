#include "module/predictor/model/outpost.hpp"
#include "utility/math/angle.hpp"
#include "utility/math/linear.hpp"
#include "utility/rclcpp/node.details.hpp"
#include "utility/robot/armor.hpp"
#include "utility/robot/constant.hpp"

#include <chrono>
#include <cmath>
#include <deque>
#include <eigen3/Eigen/Geometry>
#include <format>
#include <iostream>
#include <numbers>
#include <random>
#include <rclcpp/utilities.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace rmcs;
using namespace rmcs::util;

auto main() -> int {
    rclcpp::init(0, nullptr);

    auto node = RclcppNode { "test_outpost_ekf" };
    node.set_pub_topic_prefix("/rmcs/auto_aim/outpost_test/");

    auto marker_publisher = node.details->make_pub<visualization_msgs::msg::MarkerArray>(
        "/rmcs/auto_aim/outpost_test/ekf", qos::debug);

    const auto true_center       = Eigen::Vector3d { 2.0, 0.0, 2.0 };
    constexpr auto angular_speed = kOutpostAngularSpeed; // +0.8 pi, CCW
    constexpr auto radius        = kOutpostRadius;
    constexpr auto pitch         = kPredictedOutpostArmorPitch;
    constexpr auto dt            = 0.01;

    constexpr auto noise_sigma                = 0.30;
    constexpr auto outlier_probability        = 0.05;
    constexpr auto outlier_sigma              = 1.0;
    constexpr auto orientation_sigma          = 0.05;
    constexpr auto observation_cloud_capacity = 50;

    auto noise_engine         = std::mt19937 { 42 };
    auto noise_distribution   = std::normal_distribution<double> { 0.0, noise_sigma };
    auto outlier_distribution = std::normal_distribution<double> { 0.0, outlier_sigma };
    auto orientation_noise_distribution =
        std::normal_distribution<double> { 0.0, orientation_sigma };
    auto uniform_distribution = std::uniform_real_distribution<double> { 0.0, 1.0 };

    auto model             = std::unique_ptr<OutpostModel> { };
    auto elapsed           = 0.0;
    auto frame_index       = 0;
    auto observation_cloud = std::deque<Eigen::Vector3d> { };

    const auto start_time = std::chrono::steady_clock::now();

    while (rclcpp::ok()) {
        elapsed += dt;
        const auto stamp = node.details->rclcpp->now();

        const auto angle         = angular_speed * elapsed;
        const auto true_position = Eigen::Vector3d {
            true_center.x() + radius * std::cos(angle),
            true_center.y() + radius * std::sin(angle),
            true_center.z(),
        };
        const auto direction = true_position - true_center;
        const auto yaw_rotation =
            Eigen::AngleAxisd { std::atan2(direction.y(), direction.x()) + std::numbers::pi,
                Eigen::Vector3d::UnitZ() };
        const auto pitch_rotation   = Eigen::AngleAxisd { pitch, Eigen::Vector3d::UnitY() };
        const auto true_orientation = Eigen::Quaterniond { yaw_rotation * pitch_rotation };

        auto distance_direction = Eigen::Vector3d { true_position.normalized() };
        auto noisy_position     = Eigen::Vector3d { true_position
            + distance_direction * noise_distribution(noise_engine) };

        auto is_outlier = uniform_distribution(noise_engine) < outlier_probability;
        if (is_outlier) {
            noisy_position += distance_direction * outlier_distribution(noise_engine);
        }

        observation_cloud.push_back(noisy_position);
        if (observation_cloud.size() > observation_cloud_capacity) {
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
            model->predict(dt);
            model->correct(noisy_armor);
        }

        const auto estimated_state  = model->state();
        const auto estimated_center = Eigen::Vector3d {
            estimated_state.x,
            estimated_state.y,
            estimated_state.z,
        };
        const auto estimated_armor_position = Eigen::Vector3d {
            estimated_center.x() - radius * std::cos(estimated_state.rotation_angle),
            estimated_center.y() - radius * std::sin(estimated_state.rotation_angle),
            estimated_center.z(),
        };
        const auto estimated_orientation = Eigen::Quaterniond {
            Eigen::AngleAxisd { estimated_state.rotation_angle, Eigen::Vector3d::UnitZ() }
            * Eigen::AngleAxisd { pitch, Eigen::Vector3d::UnitY() }
        };

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

        {
            auto marker    = make_marker("true_armor");
            marker.type    = visualization_msgs::msg::Marker::CUBE;
            marker.scale.x = 0.003;
            marker.scale.y = 0.135;
            marker.scale.z = 0.056;
            marker.color.g = 1.0;
            marker.color.a = 1.0;
            set_pose(marker, true_position, true_orientation);
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

        {
            auto marker    = make_marker("estimated_armor");
            marker.type    = visualization_msgs::msg::Marker::CUBE;
            marker.scale.x = 0.003;
            marker.scale.y = 0.135;
            marker.scale.z = 0.056;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.a = 1.0;
            set_pose(marker, estimated_armor_position, estimated_orientation);
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
                    estimated_center.x() + radius * std::cos(phi),
                    estimated_center.y() + radius * std::sin(phi),
                    estimated_center.z(),
                }));
            }
            marker_array.markers.emplace_back(std::move(marker));
        }

        marker_publisher->publish(marker_array);

        if (frame_index % 10 == 0) {
            const auto center_error = (estimated_center - true_center).norm();
            const auto true_yaw     = util::normalize_angle(angle + std::numbers::pi);
            const auto angle_error =
                std::abs(util::normalize_angle(estimated_state.rotation_angle - true_yaw));
            std::cout << std::format("elapsed={:6.2f}s center_err={:.4f}m omega_est={:.4f} "
                                     "omega_true={:.4f} angle_err={:.4f}rad{}\n",
                elapsed, center_error, estimated_state.rotation_speed, angular_speed, angle_error,
                is_outlier ? " [OUTLIER]" : "");
        }

        ++frame_index;
        rclcpp::sleep_for(std::chrono::milliseconds { 10 });
    }

    return 0;
}
