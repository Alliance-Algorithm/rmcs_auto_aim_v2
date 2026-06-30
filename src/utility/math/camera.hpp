#pragma once

#include <array>
#include <opencv2/core/mat.hpp>

#include "utility/math/linear.hpp"

namespace rmcs::util {

struct CameraFeature {
    // Row Major — camera intrinsic matrix (3×3)
    std::array<std::array<double, 3>, 3> camera_matrix;

    // Distortion coefficients (k1, k2, p1, p2, k3)
    std::array<double, 5> distort_coeff;

    Orientation orientation { Orientation::kIdentity() };
    Translation translation { Translation::kZero() };

    auto from(std::array<double, 9>) -> void;
    auto from(std::array<double, 5>) -> void;

    auto cv_orientation() const -> cv::Mat;
    auto cv_translation() const -> cv::Vec3d;

    auto intrinsic() const -> cv::Mat;
    auto distortion() const -> cv::Mat;
};

auto compute_distance2cam_x(const Transform& cam, const Point3d& point) -> double;

} // namespace rmcs::util
