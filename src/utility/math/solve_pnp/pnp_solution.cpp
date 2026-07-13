#define OPENCV_DISABLE_EIGEN_TENSOR_SUPPORT
#include "pnp_solution.hpp"

#include "utility/math/angle.hpp"
#include "utility/math/conversion.hpp"
#include "utility/math/reprojection.hpp"
#include "utility/robot/constant.hpp"
#include "utility/robot/rune.hpp"

#include <eigen3/Eigen/Geometry>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <ranges>
#include <stdexcept>
#include <vector>

using namespace rmcs;
using namespace rmcs::util;

auto PnpSolution::solve() -> bool {
    try {
        const auto camera_matrix = input.camera.intrinsic();
        const auto distort_coeff = input.camera.distortion();

        const auto armor_shape = std::ranges::to<std::vector>(input.armor_shape
            | std::views::transform(
                [](const Point3d& point) { return point.make<cv::Point3f>(); }));

        const auto armor_detection = std::ranges::to<std::vector>(input.armor_detection
            | std::views::transform(
                [](const Point2d& point) { return point.make<cv::Point2f>(); }));

        auto rota_vec = cv::Vec3d { };
        auto tran_vec = cv::Vec3d { };
        auto success  = cv::solvePnP(armor_shape, armor_detection, camera_matrix, distort_coeff,
            rota_vec, tran_vec, false, cv::SOLVEPNP_IPPE);

        if (!success) return false;

        auto tran_vec_eigen_opencv = Eigen::Vector3d { };
        cv::cv2eigen(tran_vec, tran_vec_eigen_opencv);

        auto rotation_opencv = cv::Mat { };
        cv::Rodrigues(rota_vec, rotation_opencv);

        auto rotation_eigen_opencv = Eigen::Matrix3d { };
        cv::cv2eigen(rotation_opencv, rotation_eigen_opencv);

        result.genre       = input.genre;
        result.color       = input.color;
        result.translation = opencv2ros_position(tran_vec_eigen_opencv);
        result.orientation =
            Eigen::Quaterniond { opencv2ros_rotation(rotation_eigen_opencv) }.normalized();

    } catch (cv::Exception const& e) {
        throw std::runtime_error("solve pnp throw a error:" + std::string(e.what()));
    }

    return true;
}

struct OpenCvArmorShape {
    Point3d tl;
    Point3d bl;
    Point3d br;
    Point3d tr;

    explicit OpenCvArmorShape(DeviceId id) {
        const auto large = DeviceIds::kLargeArmor().contains(id);
        const auto width = large ? kLargeArmorWidth : kSmallArmorWidth;

        tl = Point3d { -0.5 * width, -0.5 * kLightBarHeight, 0.0 };
        bl = Point3d { -0.5 * width, +0.5 * kLightBarHeight, 0.0 };
        br = Point3d { +0.5 * width, +0.5 * kLightBarHeight, 0.0 };
        tr = Point3d { +0.5 * width, -0.5 * kLightBarHeight, 0.0 };
    }

    auto as_vector() const {
        return std::vector {
            tl.make<cv::Point3f>(),
            bl.make<cv::Point3f>(),
            br.make<cv::Point3f>(),
            tr.make<cv::Point3f>(),
        };
    }
};

auto RobustPnpSolution::solve() -> bool {
    const auto& feature = input.feature;
    const auto& armor2d = input.armor2d;

    const auto camera_matrix = feature.intrinsic();
    const auto distort_coeff = feature.distortion();

    const auto opencv_shape = OpenCvArmorShape { armor2d.genre };
    const auto shape_points = opencv_shape.as_vector();
    const auto armor_points = std::vector { armor2d.tl, armor2d.bl, armor2d.br, armor2d.tr };

    auto rota_vecs = std::vector<cv::Vec3d> { };
    auto tran_vecs = std::vector<cv::Vec3d> { };
    auto errors    = std::vector<double> { };

    const auto success =
        cv::solvePnPGeneric(shape_points, armor_points, camera_matrix, distort_coeff, rota_vecs,
            tran_vecs, false, cv::SOLVEPNP_IPPE, cv::noArray(), cv::noArray(), errors);

    if (!success || rota_vecs.empty() || rota_vecs.size() != tran_vecs.size()) return false;

    const auto q_odom_camera = feature.orientation.make<Eigen::Quaterniond>();
    const auto t_odom_camera = feature.translation.make<Eigen::Vector3d>();

    auto best_armor = std::optional<Armor3d> { };
    auto best_error = std::numeric_limits<double>::max();

    for (std::size_t index = 0; index < rota_vecs.size(); ++index) {
        if (tran_vecs[index][2] <= 0.0) continue;

        auto rotation_opencv = cv::Mat { };
        cv::Rodrigues(rota_vecs[index], rotation_opencv);

        auto rotation_eigen_opencv = Eigen::Matrix3d { };
        cv::cv2eigen(rotation_opencv, rotation_eigen_opencv);

        auto translation_eigen_opencv = Eigen::Vector3d { };
        cv::cv2eigen(tran_vecs[index], translation_eigen_opencv);

        const auto q_camera_armor =
            Eigen::Quaterniond { opencv2ros_rotation(rotation_eigen_opencv) }.normalized();
        const auto t_camera_armor =
            Eigen::Vector3d { opencv2ros_position(translation_eigen_opencv) };

        const auto q_odom_armor =
            Eigen::Quaterniond { q_odom_camera * q_camera_armor }.normalized();
        const auto t_odom_armor =
            Eigen::Vector3d { q_odom_camera * t_camera_armor + t_odom_camera };

        // [剪枝] 去除背对着的装甲板
        const auto back_direction  = q_camera_armor * Eigen::Vector3d::UnitX();
        const auto camera_to_armor = t_camera_armor.normalized();
        if (back_direction.dot(camera_to_armor) <= 0.0) continue;

        // [剪枝] 去除 Odom 系下，Pitch 不合理的装甲板
        const auto expected = (armor2d.genre == ArmorGenre::OUTPOST) //
            ? kPredictedOutpostArmorPitch
            : kPredictedOtherArmorPitch;
        const auto pitch    = eulers(q_odom_armor, 2, 1, 0)[1];
        if (std::abs(pitch - expected) > util::deg2rad(40.0)) continue;

        const auto error = index < errors.size() ? errors[index] : 0.0;
        if (error >= best_error) continue;

        auto armor = Armor3d { };

        armor.translation = t_odom_armor;
        armor.orientation = q_odom_armor;

        best_armor = armor;
        best_error = error;
    }

    if (!best_armor) return false;
    best_armor->genre = armor2d.genre;
    best_armor->color = armor2d.color;

    result.armor3d = *best_armor;
    { // 重投影优化
        constexpr auto kYawOptimizeRange = 60.0;
        constexpr auto kYawOptimizeStep  = 1.0;

        auto detected_points = std::array<cv::Point2f, 4> { };
        std::ranges::copy(armor_points, detected_points.begin());

        const auto q_initial = result.armor3d.orientation.make<Eigen::Quaterniond>();
        const auto ypr       = eulers(q_initial, 2, 1, 0);

        auto origin_yaw   = ypr[0];
        auto origin_pitch = ypr[1];
        if (input.fixed_outpost_pitch && armor2d.genre == ArmorGenre::OUTPOST) {
            origin_pitch = kPredictedOutpostArmorPitch;
        }
        if (input.fixed_normal_pitch && armor2d.genre != ArmorGenre::OUTPOST) {
            origin_pitch = kPredictedOtherArmorPitch;
        }

        const auto t_odom_armor  = result.armor3d.translation.make<Eigen::Vector3d>();
        const auto q_camera_odom = q_odom_camera.inverse();

        auto solution = ReprojectionSolution<4> { };
        auto evaluate = [&](double yaw) -> std::optional<double> {
            const auto q_odom_armor = euler_to_quaternion(yaw, origin_pitch, 0.0).normalized();

            const auto q_camera_armor =
                Eigen::Quaterniond { q_camera_odom * q_odom_armor }.normalized();
            const auto t_camera_armor =
                Eigen::Vector3d { q_camera_odom * (t_odom_armor - t_odom_camera) };

            const auto r_camera_armor_opencv =
                ros2opencv_rotation(q_camera_armor.toRotationMatrix());
            const auto t_camera_armor_opencv = ros2opencv_position(t_camera_armor);

            auto camera_points = std::array<cv::Point3f, 4> { };
            for (auto&& [camera_point, local_point] :
                std::views::zip(camera_points, shape_points)) {
                const auto p_local =
                    Eigen::Vector3d { local_point.x, local_point.y, local_point.z };
                const auto p_camera =
                    Eigen::Vector3d { r_camera_armor_opencv * p_local + t_camera_armor_opencv };

                camera_point = cv::Point3f {
                    static_cast<float>(p_camera.x()),
                    static_cast<float>(p_camera.y()),
                    static_cast<float>(p_camera.z()),
                };
            }

            solution.input.camera        = feature;
            solution.input.object_points = camera_points;
            solution.input.image_points  = detected_points;
            if (!solution.solve()) return std::nullopt;

            return solution.result.error;
        };

        const auto area  = deg2rad(kYawOptimizeRange * 0.5);
        const auto step  = deg2rad(kYawOptimizeStep);
        const auto steps = static_cast<int>(std::round((area * 2.0) / step));

        // 搜索最小重投影误差
        auto optimized_yaw   = origin_yaw;
        auto optimized_error = std::numeric_limits<double>::max();
        for (int i : std::views::iota(0, steps + 1)) {
            const auto yaw = (origin_yaw - area) + i * step;

            const auto error = evaluate(yaw);
            if (error && *error < optimized_error) {
                optimized_yaw   = yaw;
                optimized_error = *error;
            }
        }

        // 三点二次插值优化后同步给结果
        if (optimized_error != std::numeric_limits<double>::max()) {
            const auto error_lower = evaluate(optimized_yaw - step);
            const auto error_upper = evaluate(optimized_yaw + step);

            if (error_lower && error_upper) {
                const auto interpolation_denominator =
                    *error_lower - 2.0 * optimized_error + *error_upper + 1e-9;
                const auto interpolated_yaw = optimized_yaw
                    + step * 0.5 * (*error_lower - *error_upper) / interpolation_denominator;

                const auto interpolated_error = evaluate(interpolated_yaw);
                if (interpolated_error && *interpolated_error < optimized_error) {
                    optimized_yaw   = interpolated_yaw;
                    optimized_error = *interpolated_error;
                }
            }
        }

        result.armor3d.orientation =
            euler_to_quaternion(optimized_yaw, origin_pitch, 0.0).normalized();
    }
    return true;
}

auto SingleRunePnpSolution::solve() -> bool {
    try {
        constexpr auto kEps = 1e-6;

        const auto camera_matrix = input.cam.intrinsic();
        const auto distort_coeff = input.cam.distortion();

        const auto fx = input.cam.camera_matrix[0][0];
        const auto fy = input.cam.camera_matrix[1][1];
        if (fx <= 0.0 || fy <= 0.0) return false;

        const auto icon   = input.icon.make<cv::Point2f>();
        const auto center = input.center.make<cv::Point2f>();

        auto corners = std::array<cv::Point2f, 4> { };
        std::ranges::copy(input.corners | std::views::transform([](const Point2d& point) {
            return point.make<cv::Point2f>();
        }),
            corners.begin());

        auto index_b      = std::size_t { 0 };
        auto index_t      = std::size_t { 0 };
        auto min_distance = std::numeric_limits<double>::max();
        auto max_distance = -std::numeric_limits<double>::max();
        for (const auto& [index, corner] : corners | std::views::enumerate) {
            const auto dx       = static_cast<double>(corner.x - icon.x);
            const auto dy       = static_cast<double>(corner.y - icon.y);
            const auto distance = std::hypot(dx, dy);

            if (distance < min_distance) {
                index_b      = static_cast<std::size_t>(index);
                min_distance = distance;
            }
            if (distance > max_distance) {
                index_t      = static_cast<std::size_t>(index);
                max_distance = distance;
            }
        }
        if (index_b == index_t) return false;

        auto remaining       = std::array<std::size_t, 2> { };
        auto remaining_count = std::size_t { 0 };
        for (std::size_t index = 0; index < corners.size(); index++) {
            if (index == index_b || index == index_t) continue;
            if (remaining_count >= remaining.size()) return false;

            remaining[remaining_count] = index;
            remaining_count++;
        }
        if (remaining_count != remaining.size()) return false;

        const auto b = corners[index_b];
        const auto t = corners[index_t];

        auto l = cv::Point2f { };
        auto r = cv::Point2f { };
        {
            const auto z_axis_raw = t - b;
            const auto z_axis_len =
                std::hypot(static_cast<double>(z_axis_raw.x), static_cast<double>(z_axis_raw.y));
            if (z_axis_len <= kEps) return false;

            const auto z_axis = cv::Point2d {
                static_cast<double>(z_axis_raw.x) / z_axis_len,
                static_cast<double>(z_axis_raw.y) / z_axis_len,
            };
            const auto y_axis = cv::Point2d { z_axis.y, -z_axis.x };

            const auto first  = corners[remaining[0]];
            const auto second = corners[remaining[1]];

            const auto first_projection = static_cast<double>(first.x - center.x) * y_axis.x
                + static_cast<double>(first.y - center.y) * y_axis.y;
            const auto second_projection = static_cast<double>(second.x - center.x) * y_axis.x
                + static_cast<double>(second.y - center.y) * y_axis.y;
            if (std::abs(first_projection - second_projection) <= kEps) return false;

            if (first_projection > second_projection) {
                l = first;
                r = second;
            } else {
                l = second;
                r = first;
            }
        }

        const auto object_points = std::ranges::to<std::vector>(
            RunePagePoints::kPoints | std::views::transform([](const Point3d& point) {
                const auto p_ros = point.make<Eigen::Vector3d>();
                const auto p_ocv = ros2opencv_position(p_ros);
                return cv::Point3f {
                    static_cast<float>(p_ocv.x()),
                    static_cast<float>(p_ocv.y()),
                    static_cast<float>(p_ocv.z()),
                };
            }));
        const auto image_points = std::vector { icon, t, l, b, r };

        auto rota_vec      = cv::Vec3d { };
        auto tran_vec      = cv::Vec3d { };
        const auto success = cv::solvePnP(object_points, image_points, camera_matrix, distort_coeff,
            rota_vec, tran_vec, false, cv::SOLVEPNP_EPNP);
        if (!success || tran_vec[2] <= 0.0) return false;

        cv::solvePnP(object_points, image_points, camera_matrix, distort_coeff, rota_vec, tran_vec,
            true, cv::SOLVEPNP_ITERATIVE);
        if (tran_vec[2] <= 0.0) return false;

        auto translation_eigen_opencv = Eigen::Vector3d { };
        cv::cv2eigen(tran_vec, translation_eigen_opencv);

        auto rotation_opencv = cv::Mat { };
        cv::Rodrigues(rota_vec, rotation_opencv);

        auto rotation_eigen_opencv = Eigen::Matrix3d { };
        cv::cv2eigen(rotation_opencv, rotation_eigen_opencv);

        result.translation = opencv2ros_position(translation_eigen_opencv);
        result.orientation =
            Eigen::Quaterniond { opencv2ros_rotation(rotation_eigen_opencv) }.normalized();

    } catch (cv::Exception const&) {
        return false;
    }
    return true;
}
