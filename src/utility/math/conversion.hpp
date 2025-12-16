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

/**
 * @brief 将三维位置从 OpenCV 坐标系转换为 ROS 坐标系。
 *
 * @param position 在 OpenCV 坐标系中的位置向量，按 (x, y, z) 排列。
 * @return Eigen::Vector3d 转换到 ROS 坐标系后的位置向量，按 (x, y, z) 排列，其分量为 [z, -x, -y]（基于输入 (x, y, z)）。
 */
static inline auto opencv2ros_position(const Eigen::Vector3d& position) -> Eigen::Vector3d {
    auto result = Eigen::Vector3d(position.z(), -position.x(), -position.y());
    return result;
}

/**
 * @brief 将 3x3 旋转矩阵从 OpenCV 坐标系转换到 ROS 坐标系。
 *
 * @param rotation_matrix 在 OpenCV 坐标系下表示的 3x3 旋转矩阵。
 * @return Eigen::Matrix3d 转换到 ROS 坐标系后的 3x3 旋转矩阵。
 */
static inline Eigen::Matrix3d opencv2ros_rotation(const Eigen::Matrix3d& rotation_matrix) {
    return kCoordTransformMatrix * rotation_matrix * kCoordTransformMatrix.transpose();
}

/**
 * @brief 将三维点从 ROS 坐标系转换到 OpenCV 坐标系。
 *
 * @param position 输入点，在 ROS 坐标系下表示（x, y, z）。
 * @return Eigen::Vector3d 在 OpenCV 坐标系下表示的三维点。
 */
static inline Eigen::Vector3d ros2opencv_position(const Eigen::Vector3d& position) {
    auto result = Eigen::Vector3d(-position.y(), -position.z(), position.x());
    return result;
}

/**
 * @brief 将 3x3 旋转矩阵从 ROS 坐标系转换为 OpenCV 坐标系。
 *
 * @param rotation_matrix ROS 坐标系下的 3x3 旋转矩阵。
 * @return Eigen::Matrix3d 转换到 OpenCV 坐标系的 3x3 旋转矩阵。
 */
static inline Eigen::Matrix3d ros2opencv_rotation(const Eigen::Matrix3d& rotation_matrix) {

    return kCoordTransformMatrix.transpose() * rotation_matrix * kCoordTransformMatrix;
}

}