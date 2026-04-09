#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>

#include "utility/math/angle.hpp"
#include "utility/math/conversion.hpp"
#include "utility/math/kalman_filter/ekf.hpp"
#include "utility/robot/armor.hpp"
#include "utility/robot/constant.hpp"

namespace rmcs::predictor {

struct OutpostEKFParameters {
    using EKF = util::EKF<6, 4>;

    static constexpr int kOutpostArmorCount       = 3;
    static constexpr int kOutpostHeightOrderCount = 6;

private:
    static constexpr std::array<std::array<int, kOutpostArmorCount>, kOutpostHeightOrderCount>
        kOutpostHeightOrders { {
            { -1, 0, +1 },
            { 0, +1, -1 },
            { +1, -1, 0 },
            { -1, +1, 0 },
            { +1, 0, -1 },
            { 0, -1, +1 },
        } };

    static constexpr auto outpost_height_order(int order_idx)
        -> std::array<int, kOutpostArmorCount> {
        auto normalized = std::clamp(order_idx, 0, kOutpostHeightOrderCount - 1);
        return kOutpostHeightOrders[normalized];
    }

public:
    static constexpr auto outpost_height_rank(int order_idx, int id) -> int {
        auto normalized_id = std::clamp(id, 0, kOutpostArmorCount - 1);
        return outpost_height_order(order_idx)[normalized_id];
    }

    // x vx y vy z a
    // x, y：前哨站旋转中心在世界坐标系下的位置
    // vx, vy：前哨站旋转中心在世界坐标系下的线速度
    // z：高度居中的那块装甲板在世界坐标系下的 z 坐标
    // a: angle，0 号槽位装甲板的 yaw 角，其余槽位在此基础上相差 2pi / 3
    static auto x(Armor3D const& armor) -> EKF::XVec {
        const auto [trans_x, trans_y, trans_z]      = armor.translation;
        const auto [quat_x, quat_y, quat_z, quat_w] = armor.orientation;
        const auto orientation = Eigen::Quaterniond { quat_w, quat_x, quat_y, quat_z };

        const auto ypr      = util::eulers(orientation);
        const auto yaw      = ypr[0];
        const auto center_x = trans_x + kOutpostRadius * std::cos(yaw);
        const auto center_y = trans_y + kOutpostRadius * std::sin(yaw);

        auto x = EKF::XVec {};
        x << center_x, 0.0, center_y, 0.0, trans_z, yaw;
        return x;
    }

    static auto P_initial_dig() -> EKF::PDig {
        auto P_dig = EKF::PDig {};
        P_dig << 1.0, 64.0, 1.0, 64.0, 1.0, 0.4;
        return P_dig;
    }

    static auto armor_yaw(EKF::XVec const& x, int id) -> double {
        return util::normalize_angle(x[5] + id * 2.0 * std::numbers::pi / kOutpostArmorCount);
    }

    static auto h_armor_z(EKF::XVec const& x, int id, int order_idx) -> double {
        return x[4] + outpost_height_rank(order_idx, id) * kOutpostArmorHeightStep;
    }

    static auto h_armor_xyz(EKF::XVec const& x, int id, int order_idx) -> Eigen::Vector3d {
        const auto phase = armor_yaw(x, id);
        const auto pos_x = x[0] - kOutpostRadius * std::cos(phase);
        const auto pos_y = x[2] - kOutpostRadius * std::sin(phase);
        const auto pos_z = h_armor_z(x, id, order_idx);
        return { pos_x, pos_y, pos_z };
    }

    static auto h(EKF::XVec const& x, int id, int order_idx) -> EKF::ZVec {
        const auto xyz = h_armor_xyz(x, id, order_idx);
        const auto ypd = util::xyz2ypd(xyz);
        const auto yaw = armor_yaw(x, id);

        auto z = EKF::ZVec {};
        z << ypd[0], ypd[1], ypd[2], yaw;
        return z;
    }

    static auto x_add(EKF::XVec const& a, EKF::XVec const& b) -> EKF::XVec {
        auto result = EKF::XVec { a + b };
        result[5]   = util::normalize_angle(result[5]);
        return result;
    }

    static auto z_subtract(EKF::ZVec const& a, EKF::ZVec const& b) -> EKF::ZVec {
        auto result = EKF::ZVec { a - b };
        result[0]   = util::normalize_angle(result[0]);
        result[1]   = util::normalize_angle(result[1]);
        result[3]   = util::normalize_angle(result[3]);
        return result;
    }

    static auto F(double dt) -> EKF::AMat {
        auto F = EKF::AMat {};
        // clang-format off
        F <<
            1, dt,  0,  0,  0,  0,
            0,  1,  0,  0,  0,  0,
            0,  0,  1, dt,  0,  0,
            0,  0,  0,  1,  0,  0,
            0,  0,  0,  0,  1,  0,
            0,  0,  0,  0,  0,  1;
        // clang-format on
        return F;
    }

    static auto Q(double dt) -> EKF::QMat {
        // 平面匀速模型中的未建模线加速度噪声，作用在 x/vx 与 y/vy
        constexpr double linear_acc_var = 10.0;
        // 中间高度装甲板 z 的随机游走噪声
        constexpr double z_mid_rw_var = 1e-3;
        // 基准装甲板 yaw 角的随机游走噪声，用于补偿固定角速度模型误差
        constexpr double angle_rw_var = 1e-3;
        const auto v1                 = linear_acc_var;
        const auto v2                 = z_mid_rw_var;
        const auto v3                 = angle_rw_var;

        const auto a = dt * dt * dt * dt / 4.0;
        const auto b = dt * dt * dt / 2.0;
        const auto c = dt * dt;

        auto Q = EKF::QMat {};
        // clang-format off
        Q << a * v1, b * v1,       0,      0,       0,       0,
             b * v1, c * v1,       0,      0,       0,       0,
                  0,      0,  a * v1, b * v1,       0,       0,
                  0,      0,  b * v1, c * v1,       0,       0,
                  0,      0,       0,      0, v2 * dt,       0,
                  0,      0,       0,      0,       0, v3 * dt;
        // clang-format on
        return Q;
    }

    static auto f(double dt, int spin_sign) -> auto {
        return [dt, spin_sign](EKF::XVec const& x) {
            EKF::XVec x_prior = x;
            const auto angular_speed =
                spin_sign >= 0 ? kOutpostAngularSpeed : -kOutpostAngularSpeed;

            x_prior[0] = x[0] + x[1] * dt;
            x_prior[1] = x[1];
            x_prior[2] = x[2] + x[3] * dt;
            x_prior[3] = x[3];
            x_prior[4] = x[4];
            x_prior[5] = util::normalize_angle(x[5] + angular_speed * dt);
            return x_prior;
        };
    }

    static auto R(Eigen::Vector3d const& xyz, Eigen::Vector3d const& ypr,
        Eigen::Vector3d const& ypd) -> EKF::RMat {
        const auto center_yaw = std::atan2(xyz[1], xyz[0]);
        const auto delta_yaw  = util::normalize_angle(ypr[0] - center_yaw);
        const auto distance   = ypd[2];

        auto R_dig = EKF::RDig {};
        // clang-format off
        R_dig << 4e-3, 4e-3, std::log(std::abs(delta_yaw) + 1.0) + 1.0,
            std::log(std::abs(distance) + 1.0) / 200.0 + 9e-2;
        // clang-format on

        return R_dig.asDiagonal();
    }

    static auto H(EKF::XVec const& x, int id, int order_idx) -> EKF::HMat {
        const auto phase     = armor_yaw(x, id);
        const auto cos_phase = std::cos(phase);
        const auto sin_phase = std::sin(phase);
        const auto dx_da     = kOutpostRadius * sin_phase;
        const auto dy_da     = -kOutpostRadius * cos_phase;

        auto H_armor_xyza = Eigen::Matrix<double, 4, 6> {};
        // clang-format off
        H_armor_xyza <<
            1, 0, 0, 0, 0, dx_da,
            0, 0, 1, 0, 0, dy_da,
            0, 0, 0, 0, 1,     0,
            0, 0, 0, 0, 0,     1;
        // clang-format on

        const auto xyz         = h_armor_xyz(x, id, order_idx);
        const auto H_armor_ypd = util::xyz2ypd_jacobian(xyz);

        Eigen::Matrix<double, 4, 4> H_armor_ypda;
        // clang-format off
        H_armor_ypda <<
            H_armor_ypd(0, 0), H_armor_ypd(0, 1), H_armor_ypd(0, 2), 0,
            H_armor_ypd(1, 0), H_armor_ypd(1, 1), H_armor_ypd(1, 2), 0,
            H_armor_ypd(2, 0), H_armor_ypd(2, 1), H_armor_ypd(2, 2), 0,
                            0,                 0,                 0, 1;
        // clang-format on

        return H_armor_ypda * H_armor_xyza;
    }
};

} // namespace rmcs::predictor
