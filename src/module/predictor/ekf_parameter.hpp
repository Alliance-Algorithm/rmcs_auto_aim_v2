#pragma once

#include <cmath>

#include "utility/math/angle.hpp"
#include "utility/math/conversion.hpp"
#include "utility/math/kalman_filter/ekf.hpp"
#include "utility/robot/armor.hpp"
#include "utility/robot/constant.hpp"
#include "utility/robot/id.hpp"

namespace rmcs::predictor {

struct EKFParameters {
    using EKF = util::EKF<11, 4>;

    static auto x(Armor3D const& armor) -> EKF::XVec {
        const auto r = radius(armor.genre);

        const auto [trans_x, trans_y, trans_z]      = armor.translation;
        const auto [quat_x, quat_y, quat_z, quat_w] = armor.orientation;
        const auto orientation = Eigen::Quaterniond { quat_w, quat_x, quat_y, quat_z };

        const auto ypr      = util::eulers(orientation);
        const double yaw    = ypr[0];
        const auto center_x = trans_x + r * std::cos(yaw);
        const auto center_y = trans_y + r * std::sin(yaw);
        const auto center_z = trans_z;

        auto x = EKF::XVec { center_x, 0, center_y, 0, center_z, 0, yaw, 0, r, 0, 0 };
        return x;
    }

    static auto P_initial_dig(DeviceId const& device) -> EKF::PDig {
        auto P_dig = EKF::PDig {};
        if (device == DeviceId::OUTPOST) {
            P_dig << 1, 64, 1, 64, 1, 81, 0.4, 100, 1e-4, 0, 0;
        } else if (device == DeviceId::BASE) {
            P_dig << 1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0;
        } else {
            P_dig << 1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1;
        }

        return P_dig;
    }

    static auto radius(DeviceId const& device) -> double {

        switch (device) {
        case DeviceId::OUTPOST:
            return kOutpostRadius;
        case DeviceId::BASE:
            return kBaseRadius;
        default:
            return kOtherRadius;
        }
    }

    static auto armor_num(DeviceId const& device) -> int {
        auto num = int {};
        if (device == DeviceId::OUTPOST || device == DeviceId::BASE) num = 3;
        else num = 4;
        return num;
    }

    static auto x_add(EKF::XVec const& a, EKF::XVec const& b) -> EKF::XVec {
        auto result = EKF::XVec { a + b };
        result(6)   = util::normalize_angle(result(6));
        return result;
    }

    static auto z_subtract(EKF::ZVec const& a, EKF::ZVec const& b) -> EKF::ZVec {
        EKF::ZVec result = (a - b);
        result(0)        = util::normalize_angle(result(0));
        result(1)        = util::normalize_angle(result(1));
        result(3)        = util::normalize_angle(result(3));
        return result;
    }

    static auto F(double dt) -> EKF::AMat {
        auto F = EKF::AMat {};
        // clang-format off
        F <<
            1, dt,  0,  0,  0,  0,  0,  0,  0,  0,  0,
            0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,
            0,  0,  1, dt,  0,  0,  0,  0,  0,  0,  0,
            0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,
            0,  0,  0,  0,  1, dt,  0,  0,  0,  0,  0,
            0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,
            0,  0,  0,  0,  0,  0,  1, dt,  0,  0,  0,
            0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,
            0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,
            0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,
            0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1;
        // clang-format on
        return F;
    }

    // Piecewise White Noise Model
    // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
    static auto Q(DeviceId const& device, double dt) -> EKF::QMat {
        double acc_var, angular_acc_var;
        if (device == DeviceId::OUTPOST) {
            acc_var         = 10;
            angular_acc_var = 0.1;
        } else {
            acc_var         = 100;
            angular_acc_var = 400;
        }

        const auto a = dt * dt * dt * dt / 4.;
        const auto b = dt * dt * dt / 2;
        const auto c = dt * dt;

        auto& v1 = acc_var;
        auto& v2 = angular_acc_var;
        auto Q   = EKF::QMat {};
        // clang-format off
        Q<<
            a * v1, b * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0,
            b * v1, c * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0,
                 0,      0, a * v1, b * v1,      0,      0,      0,      0, 0, 0, 0,
                 0,      0, b * v1, c * v1,      0,      0,      0,      0, 0, 0, 0,
                 0,      0,      0,      0, a * v1, b * v1,      0,      0, 0, 0, 0,
                 0,      0,      0,      0, b * v1, c * v1,      0,      0, 0, 0, 0,
                 0,      0,      0,      0,      0,      0, a * v2, b * v2, 0, 0, 0,
                 0,      0,      0,      0,      0,      0, b * v2, c * v2, 0, 0, 0,
                 0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0,
                 0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0,
                 0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0;
        // clang-format on
        return Q;
    }

    // 计算出装甲板中心的坐标（考虑长短轴）
    static auto h_armor_xyz(EKF::XVec const& x, int id, int armor_num) -> Eigen::Vector3d {
        // x vx y vy z vz a w r l h
        // x, y, z：装甲板旋转中心在世界坐标系下的位置
        // vx, vy, vz：装甲板旋转中心在世界坐标系下的线速度
        // a: angle,装甲板相对于旋转中心的 yaw 角
        // w: angular velocity 角速度
        // r: 装甲板中心到旋转中心的半径
        // l: 连续两次观测到的半径差 r2 - r1，用于描述装甲板切换时的半径变化
        // h: 连续两次观测到的高度差 z2 - z1，反映不同装甲板之间的竖直偏移
        auto angle = x[6];
        angle      = util::normalize_angle(angle + id * 2 * std::numbers::pi / armor_num);

        const auto use_l_h = (armor_num == 4) && (id == 1 || id == 3);
        const auto r_min = x[8], l = x[9];
        const auto r = (use_l_h) ? (r_min + l) : r_min;

        const auto center_x = x[0], center_y = x[2], z_min = x[4], h = x[10];
        const auto pos_x = center_x - r * std::cos(angle);
        const auto pos_y = center_y - r * std::sin(angle);
        const auto pos_z = (use_l_h) ? (z_min + h) : z_min;

        const auto result = Eigen::Vector3d { pos_x, pos_y, pos_z };
        return result;
    }

    static auto h(EKF::XVec const& x, int id, int armor_num) -> EKF::ZVec {
        const auto xyz = h_armor_xyz(x, id, armor_num);
        const auto ypd = util::xyz2ypd(xyz);
        auto angle     = x(6);
        const auto yaw = util::normalize_angle(angle + id * 2 * std::numbers::pi / armor_num);

        const auto result = EKF::ZVec { ypd[0], ypd[1], ypd[2], yaw };
        return result;
    };

    static auto f(double dt) -> auto {
        return [dt](EKF::XVec const& x) {
            EKF::XVec x_prior = F(dt) * x;
            const auto yaw    = x_prior[6];
            x_prior[6]        = util::normalize_angle(yaw);
            return x_prior;
        };
    }

    static auto R(Eigen::Vector3d const& xyz, Eigen::Vector3d const& ypr,
        Eigen::Vector3d const& ypd) -> EKF::RMat {

        const auto x = xyz[0], y = xyz[1];
        const auto center_yaw = std::atan2(y, x);

        const auto yaw_ypr   = ypr[0];
        const auto delta_yaw = util::normalize_angle(yaw_ypr - center_yaw);

        const auto distance = ypd[2];

        auto R_dig = EKF::RDig { 4 };
        // clang-format off
        R_dig << 4e-3, 4e-3,std::log(std::abs(delta_yaw) + 1) + 1, std::log(std::abs(distance) + 1) / 200 + 9e-2;
        // clang-format on

        auto R = EKF::RMat {};
        R      = R_dig.asDiagonal();

        return R;
    }

    static auto H(EKF::XVec const& x, int id, int armor_num) -> EKF::HMat {
        // x vx y vy z vz a w r l h
        // x, y, z：装甲板旋转中心在世界坐标系下的位置
        // vx, vy, vz：装甲板旋转中心在世界坐标系下的线速度
        // a: angle,装甲板相对于旋转中心的 yaw 角
        // w: angular velocity 角速度
        // r: 装甲板中心到旋转中心的半径
        // l: 连续两次观测到的半径差 r2 - r1，用于描述装甲板切换时的半径变化
        // h: 连续两次观测到的高度差 z2 - z1，反映不同装甲板之间的竖直偏移
        auto angle           = x[6];
        angle                = util::normalize_angle(angle + id * 2 * std::numbers::pi / armor_num);
        const auto cos_angle = std::cos(angle);
        const auto sin_angle = std::sin(angle);

        const bool use_l_h = (armor_num == 4) && (id == 1 || id == 3);
        const auto r_min = x[8], l = x[9];
        const auto r = (use_l_h) ? (r_min + l) : r_min;

        const auto dx_da = r * sin_angle;
        const auto dy_da = -r * cos_angle;

        const auto dx_dr = -cos_angle;
        const auto dy_dr = -sin_angle;
        const auto dx_dl = (use_l_h) ? (-cos_angle) : 0.;
        const auto dy_dl = (use_l_h) ? (-sin_angle) : 0.;

        const auto dz_dh = (use_l_h) ? 1. : 0;

        auto H_armor_xyza = Eigen::Matrix<double, 4, 11> {};

        // clang-format off
        H_armor_xyza<<
            1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, dx_dl,     0,
            0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, dy_dl,     0,
            0, 0, 0, 0, 1, 0,     0, 0,     0,     0, dz_dh,
            0, 0, 0, 0, 0, 0,     1, 0,     0,     0,     0;
        // clang-format on

        auto xyz         = h_armor_xyz(x, id, armor_num);
        auto H_armor_ypd = util::xyz2ypd_jacobian(xyz);

        Eigen::Matrix<double, 4, 4> H_armor_ypda;
        // clang-format off
        H_armor_ypda<<
            H_armor_ypd(0, 0), H_armor_ypd(0, 1), H_armor_ypd(0, 2), 0,
            H_armor_ypd(1, 0), H_armor_ypd(1, 1), H_armor_ypd(1, 2), 0,
            H_armor_ypd(2, 0), H_armor_ypd(2, 1), H_armor_ypd(2, 2), 0,
                            0,                 0,                 0, 1;
        // clang-format on

        return H_armor_ypda * H_armor_xyza;
    }
};
}
