#include "solve_pnp.hpp"
#include <eigen3/Eigen/Dense>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <ranges>

using namespace rmcs::util;

template <std::size_t cols, typename scale>
static auto cast_opencv_matrix(std::array<scale, cols>& source) {
    auto mat_type = int {};
    /*  */ if constexpr (std::same_as<scale, double>) {
        mat_type = CV_64FC1;
    } else if constexpr (std::same_as<scale, float>) {
        mat_type = CV_32FC1;
    } else if constexpr (std::same_as<scale, int>) {
        mat_type = CV_32SC1;
    } else {
        static_assert(false, "Unsupport mat scale type");
    }

    return cv::Mat { 1, cols, mat_type, source.data() };
}

template <std::size_t cols, std::size_t rows, typename scale>
static auto cast_opencv_matrix(std::array<std::array<scale, cols>, rows>& source) {
    auto mat_type = int {};
    /*  */ if constexpr (std::same_as<scale, double>) {
        mat_type = CV_64FC1;
    } else if constexpr (std::same_as<scale, float>) {
        mat_type = CV_32FC1;
    } else if constexpr (std::same_as<scale, int>) {
        mat_type = CV_32SC1;
    } else {
        static_assert(false, "Unsupport mat scale type");
    }

    return cv::Mat { rows, cols, mat_type, source[0].data() };
}

auto PnpSolution::solve() noexcept -> void {
    const auto camera_matrix = cast_opencv_matrix(input.camera_matrix);
    const auto distort_coeff = cast_opencv_matrix(input.distort_coeff);

    const auto armor_shape = std::ranges::to<std::vector>(input.armor_shape
        | std::views::transform([](const Point3D& point) { return point.make<cv::Point3f>(); }));

    const auto armor_detection = std::ranges::to<std::vector>(input.armor_detection
        | std::views::transform([](const Point2D& point) { return point.make<cv::Point2f>(); }));

    auto rota_vec = cv::Vec3d {};
    auto tran_vec = cv::Vec3d {};
    cv::solvePnP(armor_shape, armor_detection, camera_matrix, distort_coeff, rota_vec, tran_vec,
        false, cv::SOLVEPNP_IPPE);

    {
        result.translation.x = tran_vec[0];
        result.translation.y = tran_vec[1];
        result.translation.z = tran_vec[2];
    }
    {
        auto rotation_opencv = cv::Mat {};
        cv::Rodrigues(rota_vec, rotation_opencv);

        auto rotation_eigen = Eigen::Matrix3d {};
        rotation_eigen <<                     // Col Major
            rotation_opencv.at<double>(0, 0), // [0,0]
            rotation_opencv.at<double>(0, 1), // [0,1]
            rotation_opencv.at<double>(0, 2), // [0,2]
            rotation_opencv.at<double>(1, 0), // [1,0]
            rotation_opencv.at<double>(1, 1), // [1,1]
            rotation_opencv.at<double>(1, 2), // [1,2]
            rotation_opencv.at<double>(2, 0), // [2,0]
            rotation_opencv.at<double>(2, 1), // [2,1]
            rotation_opencv.at<double>(2, 2); // [2,2]
        result.orientation = Eigen::Quaterniond { rotation_eigen };
    }
}
