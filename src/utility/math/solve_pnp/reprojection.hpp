#pragma once

#define OPENCV_DISABLE_EIGEN_TENSOR_SUPPORT

#include <array>
#include <eigen3/Eigen/Geometry>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <ranges>
#include <span>

namespace rmcs::util {

auto compute_reprojection_error(Eigen::Vector3d const& translation_camera,
    Eigen::Matrix3d const& rotation_camera, std::span<cv::Point2f const, 4> image_corners,
    std::span<cv::Point3f const, 4> object_points, cv::Mat const& camera_matrix,
    cv::Mat const& dist_coeffs) -> double {

    const auto rotation_row_major =
        Eigen::Matrix<double, 3, 3, Eigen::RowMajor> { rotation_camera };
    const auto rotation_cv = cv::Matx33d { rotation_row_major.data() };

    auto rvec = cv::Vec3d {};
    cv::Rodrigues(rotation_cv, rvec);
    const auto tvec = cv::Vec3d { translation_camera.data() };

    auto projected     = std::array<cv::Point2f, 4> {};
    auto projected_mat = cv::Mat { 4, 1, CV_32FC2, projected.data() };
    const auto object_points_mat =
        cv::Mat { 4, 1, CV_32FC3, const_cast<cv::Point3f*>(object_points.data()) };

    cv::projectPoints(object_points_mat, rvec, tvec, camera_matrix, dist_coeffs, projected_mat);

    return std::ranges::fold_left(std::views::iota(std::size_t { 0 }, projected.size()), 0.0,
        [&](double acc, std::size_t i) {
            const auto dx = projected[i].x - image_corners[i].x;
            const auto dy = projected[i].y - image_corners[i].y;
            return acc + dx * dx + dy * dy;
        });
}

} // namespace rmcs::util
