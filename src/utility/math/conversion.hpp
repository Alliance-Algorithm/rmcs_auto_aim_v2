
#pragma once

#include <eigen3/Eigen/Geometry>

namespace rmcs::util {

static inline auto opencv2ros_position(const Eigen::Vector3d& position) -> Eigen::Vector3d {
    auto result = Eigen::Vector3d(position.z(), -position.x(), -position.y());
    return result;
}

static inline Eigen::Matrix3d opencv2ros_rotation(const Eigen::Matrix3d& rotation_matrix) {
    Eigen::Matrix3d t;
    t << 0, 0, 1, //
        -1, 0, 0, //
        0, -1, 0; //
    return t * rotation_matrix * t.transpose();
}

static inline Eigen::Vector3d ros2opencv_position(const Eigen::Vector3d& position) {
    auto result = Eigen::Vector3d(-position.y(), -position.z(), position.x());
    return result;
}

static inline Eigen::Matrix3d ros2opencv_rotation(const Eigen::Matrix3d& rotation_matrix) {
    Eigen::Matrix3d t;
    t << 0, 0, 1, //
        -1, 0, 0, //
        0, -1, 0; //
    return t.transpose() * rotation_matrix * t;
}

}
