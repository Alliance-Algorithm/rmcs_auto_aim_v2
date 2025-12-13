#include "pnp_solution.hpp"

#include "utility/math/conversion.hpp"
#include "utility/math/solve_pnp.hpp"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <ranges>

#define OPENCV_DISABLE_EIGEN_TENSOR_SUPPORT
#include <opencv2/core/eigen.hpp>

using namespace rmcs::util;
auto PnpSolution::solve() noexcept -> void {
    const auto camera_matrix = cast_opencv_matrix(input.camera_matrix);
    const auto distort_coeff = cast_opencv_matrix(input.distort_coeff);

    const auto armor_shape = std::ranges::to<std::vector>(input.armor_shape
        | std::views::transform([](const Point3d& point) { return point.make<cv::Point3f>(); }));

    const auto armor_detection = std::ranges::to<std::vector>(input.armor_detection
        | std::views::transform([](const Point2d& point) { return point.make<cv::Point2f>(); }));

    auto rota_vec = cv::Vec3d {};
    auto tran_vec = cv::Vec3d {};
    cv::solvePnP(armor_shape, armor_detection, camera_matrix, distort_coeff, rota_vec, tran_vec,
        false, cv::SOLVEPNP_IPPE);

    auto tran_vec_eigen_opencv = Eigen::Vector3d {};
    cv::cv2eigen(tran_vec, tran_vec_eigen_opencv);

    auto rotation_opencv = cv::Mat {};
    cv::Rodrigues(rota_vec, rotation_opencv);
    auto rotation_eigen_opencv = Eigen::Matrix3d {};
    cv::cv2eigen(rotation_opencv, rotation_eigen_opencv);

    result.genre       = input.genre;
    result.color       = input.color;
    result.translation = opencv2ros_position(tran_vec_eigen_opencv);
    result.orientation =
        Eigen::Quaterniond(opencv2ros_rotation(rotation_eigen_opencv)).normalized();
}
