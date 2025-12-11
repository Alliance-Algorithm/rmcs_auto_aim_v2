#include "pnp_solution.hpp"

#include "utility/math/conversion.hpp"
#include "utility/math/solve_pnp.hpp"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <fmt/core.h>
#include <opencv2/core/eigen.hpp>
#include <ranges>

using namespace rmcs::util;
auto PnpSolution::solve() noexcept -> void {
    // const auto camera_matrix = cast_opencv_matrix(input.camera_matrix);
    // const auto distort_coeff = cast_opencv_matrix(input.distort_coeff);

    auto camera_matrix = Eigen::Matrix<double, 3, 3, Eigen::RowMajor> {};
    camera_matrix << 1.722231837421459e+03, 0, 7.013056440882832e+02, 0, 1.724876404292754e+03,
        5.645821718351237e+02, 0, 0, 1;

    auto distort_coeff = Eigen::Matrix<double, 1, 5> {};
    distort_coeff << -0.064232403853946, -0.087667493884102, 0, 0, 0.792381808294582;

    auto camera_matrix_ = cv::Mat {};
    auto distort_coeff_ = cv::Mat {};
    cv::eigen2cv(camera_matrix, camera_matrix_);
    cv::eigen2cv(distort_coeff, distort_coeff_);

    const auto armor_shape = std::ranges::to<std::vector>(input.armor_shape
        | std::views::transform([](const Point3d& point) { return point.make<cv::Point3f>(); }));

    const auto armor_detection = std::ranges::to<std::vector>(input.armor_detection
        | std::views::transform([](const Point2d& point) { return point.make<cv::Point2f>(); }));

    // fmt::print("K:\n");
    // for (const auto& row : input.camera_matrix) {
    //     fmt::print("  [{:f}, {:f}, {:f}]\n", row[0], row[1], row[2]);
    // }
    // fmt::print("D: [{:f}, {:f}, {:f}, {:f}, {:f}]\n", input.distort_coeff[0],
    //     input.distort_coeff[1], input.distort_coeff[2], input.distort_coeff[3],
    //     input.distort_coeff[4]);
    //
    // std::cout << "armor_shape:\n";
    // for (const auto& p : armor_shape) {
    //     std::cout << "  (" << p.x << ", " << p.y << ", " << p.z << ")\n";
    // }
    // std::cout << "armor_detection:\n";
    // for (const auto& p : armor_detection) {
    //     std::cout << "  (" << p.x << ", " << p.y << ")\n";
    // }

    auto rota_vec = cv::Vec3d {};
    auto tran_vec = cv::Vec3d {};
    cv::solvePnP(armor_shape, armor_detection, camera_matrix_, distort_coeff_, rota_vec, tran_vec,
        false, cv::SOLVEPNP_IPPE);

    auto tran_vec_eigen_opencv = Eigen::Vector3d {};
    cv::cv2eigen(tran_vec, tran_vec_eigen_opencv);
    // tran_vec_eigen_opencv.x()  = tran_vec[0];
    // tran_vec_eigen_opencv.y()  = tran_vec[1];
    // tran_vec_eigen_opencv.z()  = tran_vec[2];

    auto rotation_opencv = cv::Mat {};
    cv::Rodrigues(rota_vec, rotation_opencv);

    auto rotation_eigen_opencv = Eigen::Matrix3d {};
    cv::cv2eigen(rotation_opencv, rotation_eigen_opencv);
    // rotation_eigen_opencv <<              // Major
    //     rotation_opencv.at<double>(0, 0), // [0,0]
    //     rotation_opencv.at<double>(0, 1), // [0,1]
    //     rotation_opencv.at<double>(0, 2), // [0,2]
    //     rotation_opencv.at<double>(1, 0), // [1,0]
    //     rotation_opencv.at<double>(1, 1), // [1,1]
    //     rotation_opencv.at<double>(1, 2), // [1,2]
    //     rotation_opencv.at<double>(2, 0), // [2,0]
    //     rotation_opencv.at<double>(2, 1), // [2,1]
    //     rotation_opencv.at<double>(2, 2); // [2,2]
    //
    // rotation_eigen_opencv.transpose();
    //
    // auto [rotation_eigen_ros, tran_vec_eigen_ros] =
    //     cv_optical_to_ros_camera_link(rotation_eigen_opencv, tran_vec_eigen_opencv);
    // auto orientation_ros = Eigen::Quaterniond(rotation_eigen_ros).normalized();

    result.genre = input.genre;
    result.color = input.color;
    // result.translation = tran_vec_eigen_ros;
    // result.orientation = orientation_ros;
    result.translation = tran_vec_eigen_opencv;
    result.orientation = Eigen::Quaterniond(rotation_eigen_opencv).normalized();
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
        visualized_armor         = std::make_unique<visual::Armor>(config);
        auto const target_config = Armor::Config {

            .rclcpp = visual_node,
            .device = result.genre,
            .camp   = result.color,
            .id     = "target_pnp_armor",
            .tf     = "camera_link",
        };
        target_armor = std::make_unique<visual::Armor>(target_config);
    }
    visualized_armor->move(result.translation, result.orientation);
    visualized_armor->update();

    auto tran        = Translation { 0, 0, 1.75 };
    auto orientation = Orientation { 0, 0.7071, 0, 0.7071 };
    target_armor->move(tran, orientation);
    target_armor->update();
}
