#define OPENCV_DISABLE_EIGEN_TENSOR_SUPPORT

#include "camera.hpp"

#include <eigen3/Eigen/Geometry>
#include <opencv2/core/eigen.hpp>

#include "utility/math/conversion.hpp"

using namespace rmcs::util;

auto CameraFeature::intrinsic() const -> cv::Mat {
    return cv::Mat(3, 3, CV_64F, const_cast<double*>(camera_matrix[0].data())).clone();
}

auto CameraFeature::distortion() const -> cv::Mat {
    return cv::Mat(1, 5, CV_64F, const_cast<double*>(distort_coeff.data())).clone();
}

auto CameraFeature::orientation() const -> cv::Mat {
    auto q_ros            = world_to_camera_orientation.make<Eigen::Quaterniond>();
    auto r_ros            = q_ros.toRotationMatrix();
    Eigen::Matrix3d r_ocv = ros2opencv_rotation(r_ros);

    cv::Mat result(3, 3, CV_64F);
    cv::eigen2cv(r_ocv, result);
    return result;
}

auto CameraFeature::translation() const -> cv::Vec3d {
    auto t_ros            = world_to_camera_translation.make<Eigen::Vector3d>();
    Eigen::Vector3d t_ocv = ros2opencv_position(t_ros);
    return { t_ocv[0], t_ocv[1], t_ocv[2] };
}
