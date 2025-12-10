#include "pnp_solution.hpp"

#include <ranges>

#include "utility/math/conversion.hpp"
#include "utility/math/solve_pnp.hpp"

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
    tran_vec_eigen_opencv.x()  = tran_vec[0];
    tran_vec_eigen_opencv.y()  = tran_vec[1];
    tran_vec_eigen_opencv.z()  = tran_vec[2];

    auto rotation_opencv = cv::Mat {};
    cv::Rodrigues(rota_vec, rotation_opencv);

    auto rotation_eigen_opencv = Eigen::Matrix3d {};
    rotation_eigen_opencv <<              // Major
        rotation_opencv.at<double>(0, 0), // [0,0]
        rotation_opencv.at<double>(0, 1), // [0,1]
        rotation_opencv.at<double>(0, 2), // [0,2]
        rotation_opencv.at<double>(1, 0), // [1,0]
        rotation_opencv.at<double>(1, 1), // [1,1]
        rotation_opencv.at<double>(1, 2), // [1,2]
        rotation_opencv.at<double>(2, 0), // [2,0]
        rotation_opencv.at<double>(2, 1), // [2,1]
        rotation_opencv.at<double>(2, 2); // [2,2]

    auto [rotation_eigen_ros, tran_vec_eigen_ros] =
        cv_optical_to_ros_camera_link(rotation_eigen_opencv, tran_vec_eigen_opencv);
    auto orientation_ros = Eigen::Quaterniond(rotation_eigen_ros).normalized();

    result.genre       = input.genre;
    result.color       = input.color;
    result.translation = tran_vec_eigen_ros;
    result.orientation = orientation_ros;
}

auto PnpSolution::visualize(RclcppNode& visual_node) noexcept -> void {
    using namespace visual;
    if (!visualized_armor) {
        auto const config = Armor::Config {

            .rclcpp = visual_node,
            .device = result.genre,
            .camp   = result.color,
            .id     = "solved_pnp_armor",
            .tf     = "camera_link",
        };
        visualized_armor = std::make_unique<visual::Armor>(config);
    }
    visualized_armor->impl_move(result.translation, result.orientation);
    visualized_armor->update();
}
