
#pragma once
#include <cassert>
#include <eigen3/Eigen/Dense>

#include "utility/math/linear.hpp"

namespace rmcs::util {

// 坐标变换公式：
//   P_C_cv  = R_cv * P_W + t_cv
//   P_C_ros = T * (R_cv * P_W + t_cv) = (T * R_cv) * P_W + (T * t_cv)
// 因此：
//   R_ros = T * R_cv
//   t_ros = T * t_cv
// 其中 T 为从 OpenCV 光学系到 ROS camera_link 的基变换矩阵。

/// 生成从源坐标系到目标坐标系的转换矩阵 T，使得 v_target = T * v_source。
/// 输入轴向量支持 linear.hpp 中的自定义三维向量或 Eigen::Vector3d，并保证为正交单位向量。
template <rmcs::translation_trait VecSrc, rmcs::translation_trait VecDst>
inline auto make_basis_transform(const VecSrc& x_src, const VecSrc& y_src, const VecSrc& z_src,
    const VecDst& x_dst, const VecDst& y_dst, const VecDst& z_dst) -> Eigen::Matrix3d {

    auto to_eigen = [](const auto& v) -> Eigen::Vector3d {
        using T = std::decay_t<decltype(v)>;
        if constexpr (rmcs::translation_object_trait<T>) {
            return { v.x(), v.y(), v.z() };
        } else {
            return { v.x, v.y, v.z };
        }
    };

    const auto Xs = to_eigen(x_src);
    const auto Ys = to_eigen(y_src);
    const auto Zs = to_eigen(z_src);
    const auto Xd = to_eigen(x_dst);
    const auto Yd = to_eigen(y_dst);
    const auto Zd = to_eigen(z_dst);

    [[maybe_unused]] auto is_unit = [](const Eigen::Vector3d& v) {
        return std::abs(v.norm() - 1.0) < 1e-6;
    };
    [[maybe_unused]] auto is_orth = [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
        return std::abs(a.dot(b)) < 1e-6;
    };

    assert(is_unit(Xs) && is_unit(Ys) && is_unit(Zs) && "source  must be unit length");
    assert(is_unit(Xd) && is_unit(Yd) && is_unit(Zd) && "target  must be unit length");
    assert(is_orth(Xs, Ys) && is_orth(Ys, Zs) && is_orth(Zs, Xs) && "source  must be orthogonal");
    assert(is_orth(Xd, Yd) && is_orth(Yd, Zd) && is_orth(Zd, Xd) && "target  must be orthogonal");

    Eigen::Matrix3d R_src;
    R_src.col(0) = Xs;
    R_src.col(1) = Ys;
    R_src.col(2) = Zs;

    Eigen::Matrix3d R_dst;
    R_dst.col(0) = Xd;
    R_dst.col(1) = Yd;
    R_dst.col(2) = Zd;

    // 将源坐标的分量映射到目标坐标分量
    return R_dst.transpose() * R_src;
}

/// OpenCV 光学坐标系 (x:右, y:下, z:前) -> ROS camera_link (x:前, y:左, z:上)
inline auto make_cv_optical_to_ros_camera_link() -> Eigen::Matrix3d {
    using V = Eigen::Vector3d;
    const V x_cv { 1., 0., 0. };
    const V y_cv { 0., 1., 0. };
    const V z_cv { 0., 0., 1. };

    const V x_ros { 0., 0., 1. };  // 前
    const V y_ros { -1., 0., 0. }; // 左
    const V z_ros { 0., -1., 0. }; // 上

    return make_basis_transform(x_cv, y_cv, z_cv, x_ros, y_ros, z_ros);
}

/// 将 solvePnP 的输出 (OpenCV 光学坐标系) 转换为 ROS camera_link 坐标系
inline auto cv_optical_to_ros_camera_link(const Eigen::Matrix3d& R_cv, const Eigen::Vector3d& t_cv)
    -> std::pair<Eigen::Matrix3d, Eigen::Vector3d> {
    const auto T = make_cv_optical_to_ros_camera_link();
    return { T * R_cv, T * t_cv };
}

}
