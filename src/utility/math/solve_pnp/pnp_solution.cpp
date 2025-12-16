#include "pnp_solution.hpp"

#include <eigen3/Eigen/Dense>
#include <ranges>
#define OPENCV_DISABLE_EIGEN_TENSOR_SUPPORT
#include <opencv2/core/eigen.hpp>

#include "utility/math/conversion.hpp"
#include "utility/math/solve_pnp/solve_pnp.hpp"

using namespace rmcs::util;
/**
 * @brief 基于输入的三维点与对应的二维图像点及相机参数，执行 PnP 求解并填充 result 的位姿信息。
 *
 * 将输入的 armor_shape、armor_detection 以及相机内参转换为 OpenCV 格式，使用 IPPE 求解旋转与平移，
 * 并将结果转换为 ROS 风格的位置与归一化四元数赋值到 result 中。
 *
 * @return true 如果求解成功且 result 已被填充；false 如果 cv::solvePnP 未能求解出位姿。
 *
 * @throws std::runtime_error 当 OpenCV 抛出 cv::Exception 时，将其信息封装并以该异常类型重新抛出。
 */
auto PnpSolution::solve() -> bool {
    try {
        const auto camera_matrix = cast_opencv_matrix(input.camera_matrix);
        const auto distort_coeff = cast_opencv_matrix(input.distort_coeff);

        const auto armor_shape = std::ranges::to<std::vector>(input.armor_shape
            | std::views::transform(
                [](const Point3d& point) { return point.make<cv::Point3f>(); }));

        const auto armor_detection = std::ranges::to<std::vector>(input.armor_detection
            | std::views::transform(
                [](const Point2d& point) { return point.make<cv::Point2f>(); }));

        auto rota_vec = cv::Vec3d {};
        auto tran_vec = cv::Vec3d {};
        auto success  = cv::solvePnP(armor_shape, armor_detection, camera_matrix, distort_coeff,
             rota_vec, tran_vec, false, cv::SOLVEPNP_IPPE);

        if (!success) return false;

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
            Eigen::Quaterniond { opencv2ros_rotation(rotation_eigen_opencv) }.normalized();
    } catch (cv::Exception const& e) {
        throw std::runtime_error("solve pnp throw a error:" + std::string(e.what()));
    }

    return true;
}