#define OPENCV_DISABLE_EIGEN_TENSOR_SUPPORT

#include "outpost_distance_optimizer.hpp"

#include "utility/math/conversion.hpp"
#include "utility/math/outpost.hpp"

#include <array>
#include <cmath>
#include <vector>

#include <eigen3/Eigen/Geometry>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

using namespace rmcs::util;

auto OutpostDistanceOptimizer::solve() -> bool {

    auto q_ac_ros = input.initial.orientation.make<Eigen::Quaterniond>();
    auto t_ac_ros = input.initial.translation.make<Eigen::Vector3d>();
    auto r_ac_ros = q_ac_ros.toRotationMatrix();

    auto object_points = std::vector<cv::Point3f> { };
    object_points.reserve(6);
    for (const auto index : { 0, 2, 1, 3 }) {
        const auto& point    = kSmallArmorShapeRos[index];
        const auto point_ocv = ros2opencv_position(point.make<Eigen::Vector3d>());
        object_points.emplace_back(static_cast<float>(point_ocv.x()),
            static_cast<float>(point_ocv.y()), static_cast<float>(point_ocv.z()));
    }

    auto camera_matrix = input.camera.intrinsic();
    auto distort_coeff = input.camera.distortion();
    auto r_ac_ocv      = ros2opencv_rotation(r_ac_ros);
    auto t_ac_ocv      = ros2opencv_position(t_ac_ros);
    auto r_ac_ocv_cv   = cv::Mat { };
    cv::eigen2cv(r_ac_ocv, r_ac_ocv_cv);

    auto initial_rvec = cv::Vec3d { };
    auto initial_tvec = cv::Vec3d { t_ac_ocv.x(), t_ac_ocv.y(), t_ac_ocv.z() };
    cv::Rodrigues(r_ac_ocv_cv, initial_rvec);

    auto solution                  = NeighborBarSolution { };
    solution.input.source          = input.initial;
    solution.input.in_right        = input.is_right;
    solution.input.armor_thickness = input.armor_thickness;
    solution.solve();

    const auto& selected_lightbar =
        input.is_upper ? solution.result.upper_near : solution.result.lower_near;

    for (const auto& point : { selected_lightbar.first, selected_lightbar.second }) {
        const auto point_in_camera_ros = point.make<Eigen::Vector3d>();
        auto point_in_local_ros =
            Eigen::Vector3d { (r_ac_ros.transpose() * (point_in_camera_ros - t_ac_ros)).eval() };

        const auto point_in_local_ocv = ros2opencv_position(point_in_local_ros);
        object_points.emplace_back(static_cast<float>(point_in_local_ocv.x()),
            static_cast<float>(point_in_local_ocv.y()), static_cast<float>(point_in_local_ocv.z()));
    }

    const auto image_points = std::vector {
        input.armor.tl,
        input.armor.tr,
        input.armor.br,
        input.armor.bl,
        input.upper_point.make<cv::Point2f>(),
        input.lower_point.make<cv::Point2f>(),
    };

    auto optimized_rvec = cv::Vec3d { initial_rvec };
    auto optimized_tvec = cv::Vec3d { initial_tvec };
    const auto solved   = cv::solvePnP(object_points, image_points, camera_matrix, distort_coeff,
        optimized_rvec, optimized_tvec, true, cv::SOLVEPNP_ITERATIVE);
    if (!solved || optimized_tvec[2] <= 0.0) return false;

    auto optimized_rotation_cv = cv::Mat { };
    cv::Rodrigues(optimized_rvec, optimized_rotation_cv);

    auto optimized_rotation_ocv = Eigen::Matrix3d { };
    cv::cv2eigen(optimized_rotation_cv, optimized_rotation_ocv);

    auto optimized_translation_ocv = Eigen::Vector3d { };
    cv::cv2eigen(optimized_tvec, optimized_translation_ocv);

    result.armor             = input.initial;
    result.armor.translation = opencv2ros_position(optimized_translation_ocv);
    result.armor.orientation =
        Eigen::Quaterniond { opencv2ros_rotation(optimized_rotation_ocv) }.normalized();

    return true;
}
