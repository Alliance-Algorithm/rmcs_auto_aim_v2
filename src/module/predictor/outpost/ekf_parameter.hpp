#pragma once

#include "module/predictor/outpost/armor_layout.hpp"
#include "utility/math/angle.hpp"
#include "utility/math/conversion.hpp"
#include "utility/math/kalman_filter/ekf.hpp"
#include "utility/panic.hpp"
#include "utility/robot/armor.hpp"
#include "utility/robot/constant.hpp"

#include <array>
#include <cmath>
#include <numbers>

namespace rmcs::predictor {

struct OutpostEKFParameters {
    using EKF = util::EKF<7, 4>;

    struct Measurement {
        EKF::ZVec z;
        EKF::RMat R;
    };

    static constexpr int kOutpostArmorCount = 3;
    static constexpr double kPhaseStep      = 2.0 * std::numbers::pi / kOutpostArmorCount;

    // x vx y vy z a w
    // x, y：前哨站旋转中心在世界坐标系下的位置
    // vx, vy：前哨站旋转中心在世界坐标系下的线速度
    // z：参考装甲板(id 0)在世界坐标系下的 z 坐标
    // a：参考装甲板(id 0)的 yaw 角
    // w：参考装甲板(id 0)的角速度
    static auto x(Armor3d const& armor) -> EKF::XVec {
        const auto [trans_x, trans_y, trans_z]      = armor.translation;
        const auto [quat_x, quat_y, quat_z, quat_w] = armor.orientation;
        const auto orientation = Eigen::Quaterniond { quat_w, quat_x, quat_y, quat_z };

        const auto ypr      = util::eulers(orientation);
        const auto yaw      = ypr[0];
        const auto center_x = trans_x + kOutpostRadius * std::cos(yaw);
        const auto center_y = trans_y + kOutpostRadius * std::sin(yaw);

        auto x = EKF::XVec {};
        x << center_x, 0.0, center_y, 0.0, trans_z, yaw, 0.0;
        return x;
    }

    static auto P_initial_dig() -> EKF::PDig {
        auto P_dig = EKF::PDig {};
        P_dig << 1.0, 64.0, 1.0, 64.0, 1.0, 0.4, 16.0;
        return P_dig;
    }

    static auto armor_yaw(EKF::XVec const& x, double phase_offset) -> double {
        return util::normalize_angle(x[5] + phase_offset);
    }

    static auto phase_offset(int armor_id) -> double {
        constexpr auto kPhaseOffsets =
            std::array<double, kOutpostArmorCount> { 0.0, kPhaseStep, -kPhaseStep };
        if (armor_id < 0 || armor_id >= kOutpostArmorCount) {
            util::panic("outpost armor id out of range");
        }
        return kPhaseOffsets[static_cast<std::size_t>(armor_id)];
    }

    static auto armor_yaw(EKF::XVec const& x, int armor_id) -> double {
        return armor_yaw(x, phase_offset(armor_id));
    }

    static auto height_offset(int height_level) -> double {
        using rmcs::kOutpostArmorHeightStep;
        return height_level * kOutpostArmorHeightStep;
    }

    static auto height_offset(OutpostArmorLayout const& layout, int armor_id) -> double {
        if (armor_id < 0 || armor_id >= kOutpostArmorCount) {
            util::panic("outpost armor id out of range");
        }
        auto const height_level = layout.height_levels[static_cast<std::size_t>(armor_id)];
        if (!height_level.has_value()) {
            util::panic("outpost armor height level is unknown");
        }
        return height_offset(*height_level);
    }

    static auto h_armor_xyz(EKF::XVec const& x, double phase_offset, double height_offset)
        -> Eigen::Vector3d {
        const auto phase = armor_yaw(x, phase_offset);
        const auto pos_x = x[0] - kOutpostRadius * std::cos(phase);
        const auto pos_y = x[2] - kOutpostRadius * std::sin(phase);
        const auto pos_z = x[4] + height_offset;
        return { pos_x, pos_y, pos_z };
    }

    static auto h(EKF::XVec const& x, double phase_offset, double height_offset) -> EKF::ZVec {
        const auto xyz = h_armor_xyz(x, phase_offset, height_offset);
        const auto ypd = util::xyz2ypd(xyz);
        const auto yaw = armor_yaw(x, phase_offset);

        auto z = EKF::ZVec {};
        z << ypd[0], ypd[1], ypd[2], yaw;
        return z;
    }

    static auto h(EKF::XVec const& x, Armor3d const& armor, double height_offset) -> EKF::ZVec {
        return h(x, phase_offset(armor.id), height_offset);
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
            1, dt,  0,  0,  0,  0,  0,
            0,  1,  0,  0,  0,  0,  0,
            0,  0,  1, dt,  0,  0,  0,
            0,  0,  0,  1,  0,  0,  0,
            0,  0,  0,  0,  1,  0,  0,
            0,  0,  0,  0,  0,  1, dt,
            0,  0,  0,  0,  0,  0,  1;
        // clang-format on
        return F;
    }

    static auto Q(double dt) -> EKF::QMat {
        // 平面匀速模型中的未建模线加速度噪声，作用在 x/vx 与 y/vy
        constexpr double linear_acc_var = 10.0;
        // 参考装甲板 z 的随机游走噪声
        constexpr double z_ref_rw_var = 1e-3;
        // 参考装甲板 yaw / 角速度的角加速度噪声
        constexpr double angular_acc_var = 4.0;
        const auto v1                    = linear_acc_var;
        const auto v2                    = z_ref_rw_var;
        const auto v3                    = angular_acc_var;

        const auto a = dt * dt * dt * dt / 4.0;
        const auto b = dt * dt * dt / 2.0;
        const auto c = dt * dt;

        auto Q = EKF::QMat {};
        // clang-format off
        Q << a * v1, b * v1,       0,      0,       0,       0,       0,
             b * v1, c * v1,       0,      0,       0,       0,       0,
                  0,      0,  a * v1, b * v1,       0,       0,       0,
                  0,      0,  b * v1, c * v1,       0,       0,       0,
                  0,      0,       0,      0, v2 * dt,       0,       0,
                  0,      0,       0,      0,       0,  a * v3, b * v3,
                  0,      0,       0,      0,       0,  b * v3, c * v3;
        // clang-format on
        return Q;
    }

    static auto f(double dt) -> auto {
        return [dt](EKF::XVec const& x) {
            EKF::XVec x_prior = F(dt) * x;
            x_prior[5]        = util::normalize_angle(x_prior[5]);
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

    static auto measurement(Armor3d const& armor) -> Measurement {
        auto const [pos_x, pos_y, pos_z] = armor.translation;
        auto const xyz                   = Eigen::Vector3d { pos_x, pos_y, pos_z };

        auto const [quat_x, quat_y, quat_z, quat_w] = armor.orientation;
        auto const orientation = Eigen::Quaterniond { quat_w, quat_x, quat_y, quat_z };

        auto const ypr = util::eulers(orientation);
        auto const ypd = util::xyz2ypd(xyz);

        auto z = EKF::ZVec {};
        z << ypd[0], ypd[1], ypd[2], ypr[0];

        return { z, R(xyz, ypr, ypd) };
    }

    static auto H(EKF::XVec const& x, double phase_offset, double height_offset) -> EKF::HMat {
        const auto phase     = armor_yaw(x, phase_offset);
        const auto cos_phase = std::cos(phase);
        const auto sin_phase = std::sin(phase);
        const auto dx_da     = kOutpostRadius * sin_phase;
        const auto dy_da     = -kOutpostRadius * cos_phase;

        auto H_armor_xyza = Eigen::Matrix<double, 4, 7> {};
        // clang-format off
        H_armor_xyza <<
            1, 0, 0, 0, 0, dx_da, 0,
            0, 0, 1, 0, 0, dy_da, 0,
            0, 0, 0, 0, 1,     0, 0,
            0, 0, 0, 0, 0,     1, 0;
        // clang-format on

        const auto xyz         = h_armor_xyz(x, phase_offset, height_offset);
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

    static auto H(EKF::XVec const& x, Armor3d const& armor, double height_offset) -> EKF::HMat {
        return H(x, phase_offset(armor.id), height_offset);
    }
};

} // namespace rmcs::predictor
