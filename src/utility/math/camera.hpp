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

    // World-to-camera extrinsic transform (ROS convention)
    //   p_camera = quaternion * p_world + translation
    Orientation camera_orientation { Orientation::kIdentity() };
    Translation camera_translation { Translation::kZero() };

    auto orientation() const -> cv::Mat;
    auto translation() const -> cv::Vec3d;
    auto intrinsic() const -> cv::Mat;
    auto distortion() const -> cv::Mat;
};

} // namespace rmcs::util
