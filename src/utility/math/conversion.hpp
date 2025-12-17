#pragma once
#include <eigen3/Eigen/Geometry>

namespace rmcs::util {

// OpenCV 与 ROS 坐标系之间的变换矩阵
inline const Eigen::Matrix3d kCoordTransformMatrix {
    { +0, +0, +1 },
    { -1, +0, +0 },
    { +0, -1, +0 },
};

inline auto opencv2ros_position(const Eigen::Vector3d& position) -> Eigen::Vector3d {
    auto result = Eigen::Vector3d { position.z(), -position.x(), -position.y() };
    return result;
}
inline auto opencv2ros_rotation(const Eigen::Matrix3d& rotation_matrix) -> Eigen::Matrix3d {
    return kCoordTransformMatrix * rotation_matrix * kCoordTransformMatrix.transpose();
}
inline auto ros2opencv_position(const Eigen::Vector3d& position) -> Eigen::Vector3d {
    auto result = Eigen::Vector3d { -position.y(), -position.z(), position.x() };
    return result;
}
inline auto ros2opencv_rotation(const Eigen::Matrix3d& rotation_matrix) -> Eigen::Matrix3d {
    return kCoordTransformMatrix.transpose() * rotation_matrix * kCoordTransformMatrix;
}

}
