
#pragma once

#include <eigen3/Eigen/Geometry>

namespace rmcs::util {

// OpenCV 与 ROS 坐标系之间的变换矩阵
inline const Eigen::Matrix3d kCoordTransformMatrix {
    // clang-format off
    { 0, 0, 1 }, 
    { -1, 0, 0 },
    { 0, -1, 0 }

    // clang-format on
};

static inline auto opencv2ros_position(const Eigen::Vector3d& position) -> Eigen::Vector3d {
    auto result = Eigen::Vector3d(position.z(), -position.x(), -position.y());
    return result;
}

static inline Eigen::Matrix3d opencv2ros_rotation(const Eigen::Matrix3d& rotation_matrix) {
    return kCoordTransformMatrix * rotation_matrix * kCoordTransformMatrix.transpose();
}

static inline Eigen::Vector3d ros2opencv_position(const Eigen::Vector3d& position) {
    auto result = Eigen::Vector3d(-position.y(), -position.z(), position.x());
    return result;
}

static inline Eigen::Matrix3d ros2opencv_rotation(const Eigen::Matrix3d& rotation_matrix) {

    return kCoordTransformMatrix.transpose() * rotation_matrix * kCoordTransformMatrix;
}

}
