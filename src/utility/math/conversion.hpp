#pragma once
#include "utility/math/angle.hpp"
#include <eigen3/Eigen/Geometry>
#include <opencv2/core/cvdef.h>

namespace rmcs::util {

// OpenCV 与 ROS 坐标系之间的变换矩阵
inline const Eigen::Matrix3d kCoordTransformMatrix {
    { +0, +0, +1 },
    { -1, +0, +0 },
    { +0, -1, +0 },
};

inline constexpr auto opencv2ros_position(const Eigen::Vector3d& position) -> Eigen::Vector3d {
    auto result = Eigen::Vector3d { position.z(), -position.x(), -position.y() };
    return result;
}
inline constexpr auto opencv2ros_rotation(const Eigen::Matrix3d& rotation_matrix)
    -> Eigen::Matrix3d {
    return kCoordTransformMatrix * rotation_matrix * kCoordTransformMatrix.transpose();
}
inline constexpr auto ros2opencv_position(const Eigen::Vector3d& position) -> Eigen::Vector3d {
    auto result = Eigen::Vector3d { -position.y(), -position.z(), position.x() };
    return result;
}
inline constexpr auto ros2opencv_rotation(const Eigen::Matrix3d& rotation_matrix)
    -> Eigen::Matrix3d {
    return kCoordTransformMatrix.transpose() * rotation_matrix * kCoordTransformMatrix;
}

inline constexpr auto xyz2ypd(Eigen::Vector3d const& xyz) -> Eigen::Vector3d {
    const auto x = xyz[0];
    const auto y = xyz[1];
    const auto z = xyz[2];

    const auto yaw      = std::atan2(y, x);
    const auto pitch    = std::atan2(z, std::sqrt(x * x + y * y));
    const auto distance = std::sqrt(x * x + y * y + z * z);
    const auto result   = Eigen::Vector3d { yaw, pitch, distance };
    return result;
}

constexpr auto xyz2ypd_jacobian(Eigen::Vector3d const& xyz) -> auto {
    const auto x = xyz[0];
    const auto y = xyz[1];
    const auto z = xyz[2];

    const auto dyaw_dx = -y / (x * x + y * y);
    const auto dyaw_dy = x / (x * x + y * y);
    const auto dyaw_dz = 0.;

    const auto dpitch_dx =
        -(x * z) / ((z * z / (x * x + y * y) + 1) * std::pow((x * x + y * y), 1.5));
    const auto dpitch_dy =
        -(y * z) / ((z * z / (x * x + y * y) + 1) * std::pow((x * x + y * y), 1.5));
    const auto dpitch_dz = 1 / ((z * z / (x * x + y * y) + 1) * std::pow((x * x + y * y), 0.5));

    const auto ddistance_dx = x / std::pow((x * x + y * y + z * z), 0.5);
    const auto ddistance_dy = y / std::pow((x * x + y * y + z * z), 0.5);
    const auto ddistance_dz = z / std::pow((x * x + y * y + z * z), 0.5);

    auto J = Eigen::Matrix<double, 3, 3> {};
    // clang-format off
    J<<    dyaw_dx,     dyaw_dy,     dyaw_dz,
         dpitch_dx,   dpitch_dy,   dpitch_dz,
      ddistance_dx,ddistance_dy,ddistance_dz;
    // clang-format on

    return J;
}

constexpr auto eulers(Eigen::Quaterniond const& q, int axis0 = 2, int axis1 = 1, int axis2 = 0,
    bool extrinsic = false) -> Eigen::Vector3d {
    if (!extrinsic) std::swap(axis0, axis2);

    auto i = axis0, j = axis1, k = axis2;
    auto is_proper = (i == k);
    if (is_proper) k = 3 - i - j;
    auto sign = (i - j) * (j - k) * (k - i) / 2;

    double a, b, c, d;
    Eigen::Vector4d xyzw = q.coeffs();
    if (is_proper) {
        a = xyzw[3];
        b = xyzw[i];
        c = xyzw[j];
        d = xyzw[k] * sign;
    } else {
        a = xyzw[3] - xyzw[j];
        b = xyzw[i] + xyzw[k] * sign;
        c = xyzw[j] + xyzw[3];
        d = xyzw[k] * sign - xyzw[i];
    }

    const auto n2 = a * a + b * b + c * c + d * d;
    auto eulers   = Eigen::Vector3d {};
    eulers[1]     = std::acos(2 * (a * a + b * b) / n2 - 1);

    const auto half_sum  = std::atan2(b, a);
    const auto half_diff = std::atan2(-d, c);

    const auto eps   = 1e-7;
    const auto safe1 = std::abs(eulers[1]) >= eps;
    const auto safe2 = std::abs(eulers[1] - CV_PI) >= eps;
    const auto safe  = safe1 && safe2;
    if (safe) {
        eulers[0] = half_sum + half_diff;
        eulers[2] = half_sum - half_diff;
    } else {
        if (!extrinsic) {
            eulers[0] = 0;
            if (!safe1) eulers[2] = 2 * half_sum;
            if (!safe2) eulers[2] = -2 * half_diff;
        } else {
            eulers[2] = 0;
            if (!safe1) eulers[0] = 2 * half_sum;
            if (!safe2) eulers[0] = 2 * half_diff;
        }
    }

    for (int i = 0; i < 3; i++)
        eulers[i] = normalize_angle(eulers[i]);

    if (!is_proper) {
        eulers[2] *= sign;
        eulers[1] -= CV_PI / 2;
    }

    if (!extrinsic) std::swap(eulers[0], eulers[2]);

    return eulers;
};

constexpr auto ypd2xyz(const Eigen::Vector3d& ypd) -> Eigen::Vector3d {
    const auto yaw = ypd[0], pitch = ypd[1], distance = ypd[2];
    const auto x = distance * std::cos(pitch) * std::cos(yaw);
    const auto y = distance * std::cos(pitch) * std::sin(yaw);
    const auto z = distance * std::sin(pitch);
    return { x, y, z };
}

}
