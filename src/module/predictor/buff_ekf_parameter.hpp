#pragma once

#include <algorithm>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <type_traits>

#include "utility/math/angle.hpp"
#include "utility/math/conversion.hpp"
#include "utility/math/kalman_filter/ekf.hpp"
#include "utility/robot/constant.hpp"
#include "utility/robot/fan_blade.hpp"

namespace rmcs::predictor {
using SmallEKF = util::EKF<7, 7>;
using LargeEKF = util::EKF<9, 7>;

struct BuffEKFParameters {
    static constexpr int kIdxRuneYaw         = 4;
    static constexpr int kIdxBladePhase      = 5;
    static constexpr int kIdxSmallBladeSpeed = 6;

private:
    template <typename T>
    static constexpr bool is_small_ekf_v =
        std::decay_t<T>::XVec::RowsAtCompileTime == SmallEKF::XVec::RowsAtCompileTime;

    template <typename T>
    static constexpr bool is_large_ekf_v =
        std::decay_t<T>::XVec::RowsAtCompileTime == LargeEKF::XVec::RowsAtCompileTime;

    static auto clamp_large_shape_states(LargeEKF::XVec& x) -> void {
        x(6) = std::clamp(x(6), kLargeRuneAmplitudeMin, kLargeRuneAmplitudeMax);
        x(7) = std::clamp(x(7), kLargeRuneOmegaMin, kLargeRuneOmegaMax);
    }

    // Small EKF state:
    // x_s = [R_yaw, R_yaw_w, R_pitch, R_dist, rune_yaw, blade_phase, blade_w]
    // Measurement (small/large shared):
    // z = [R_yaw, R_pitch, R_dist, blade_phase, B_yaw, B_pitch, B_dist]
    static auto small_x(FanBlade3D const& blade, int clockwise) -> SmallEKF::XVec {
        const auto [quat_x, quat_y, quat_z, quat_w] = blade.orientation;
        const auto orientation = Eigen::Quaterniond { quat_w, quat_x, quat_y, quat_z };

        auto R_mark_xyz = Eigen::Vector3d {};
        blade.R_mark_position.copy_to(R_mark_xyz);

        const auto R_mark_ypd      = util::xyz2ypd(R_mark_xyz);
        const auto R_mark_yaw      = R_mark_ypd[0];
        const auto R_mark_pitch    = R_mark_ypd[1];
        const auto R_mark_distance = R_mark_ypd[2];

        const auto rune_ypr  = util::eulers(orientation);
        const auto rune_yaw  = rune_ypr[0];
        const auto rune_roll = rune_ypr[2];
        const auto x = SmallEKF::XVec { R_mark_yaw, 0., R_mark_pitch, R_mark_distance, rune_yaw,
            rune_roll, kSmallRuneAngularVelocity * clockwise };
        return x;
    }

    static auto small_f(double dt, SmallEKF::XVec const& x) -> SmallEKF::XVec {
        // x_{k+1} = F(dt) * x_k
        // R_yaw' = R_yaw + R_yaw_w * dt
        // blade_phase' = blade_phase + blade_w * dt
        SmallEKF::XVec x_prior = small_F(dt) * x;
        // Periodic states are normalized after propagation.
        x_prior(0) = util::normalize_angle(x_prior(0));
        x_prior(2) = util::normalize_angle(x_prior(2));
        x_prior(4) = util::normalize_angle(x_prior(4));
        x_prior(5) = util::normalize_angle(x_prior(5));
        return x_prior;
    }

    static auto small_F(double dt) -> SmallEKF::AMat {
        auto F = SmallEKF::AMat {};
        // x' = F x, non-identity terms:
        // F(0,1)=dt  => R_mark_yaw' = R_mark_yaw + R_mark_angular_velocity * dt
        // F(5,6)=dt  => blade_phase' = blade_phase + blade_w*dt
        // clang-format off
        F << 1.0,  dt, 0.0, 0.0, 0.0, 0.0, 0.0, // R_mark_yaw += R_mark_angular_velocity * dt
             0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, // R_mark_angular_velocity
             0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, // R_mark_pitch
             0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, // R_mark_distance
             0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, // rune_yaw
             0.0, 0.0, 0.0, 0.0, 0.0, 1.0,  dt, // blade_angle += blade_speed * dt
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0; // blade_speed
        // clang-format on
        return F;
    }

    static auto small_Q(double dt) -> SmallEKF::QMat {
        const auto R_mark_yaw_acc_var = 0.001; // R标中心偏航角加速度方差
        const auto a                  = dt * dt * dt * dt / 4;
        const auto b                  = dt * dt * dt / 2;
        const auto c                  = dt * dt;

        // Constant-acceleration process noise on [R_yaw, R_yaw_w]:
        // Q_yaw = σ_a^2 * [[dt^4/4, dt^3/2],
        //                  [dt^3/2, dt^2  ]]
        auto Q = SmallEKF::QMat {};
        // clang-format off
        Q << a * R_mark_yaw_acc_var, b * R_mark_yaw_acc_var, 0.0, 0.0, 0.0, 0.0, 0.0,
             b * R_mark_yaw_acc_var, c * R_mark_yaw_acc_var, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0,                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0,                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0,                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0,                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0,                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        // clang-format on
        return Q;
    }

    static auto small_P() -> SmallEKF::PMat {
        auto P = SmallEKF::PMat {};
        // R_mark_yaw, R_mark_angular_velocity, R_mark_pitch, R_mark_distance, rune_yaw,
        // blade_angle, blade_speed
        P.diagonal() << 10., 10., 10., 10., 10., 10., 1e-2;
        return P;
    }

    // Shared measurement model is defined on the first 6 state dimensions:
    // [R_yaw, R_yaw_w, R_pitch, R_dist, rune_yaw, blade_phase].
    using ObservationSubstate = Eigen::Matrix<double, 6, 1>;
    using ObservationSubHMat  = Eigen::Matrix<double, 7, 6>;

    template <typename XVec>
    static auto observation_substate(XVec const& x) -> ObservationSubstate {
        static_assert(XVec::RowsAtCompileTime >= 6, "state vector must have at least 6 dimensions");
        ObservationSubstate result = ObservationSubstate::Zero();
        result                     = x.template head<6>();
        return result;
    }

    static auto shared_H(ObservationSubstate const& x) -> ObservationSubHMat {
        ObservationSubHMat H = ObservationSubHMat::Zero();

        // Linear observation:
        // z0 = x0, z1 = x2, z2 = x3, z3 = x5.
        H(0, 0) = 1.0;
        H(1, 2) = 1.0;
        H(2, 3) = 1.0;
        H(3, 5) = 1.0;

        const auto R_mark_yaw = x(0), R_mark_pitch = x(2), R_mark_distance = x(3);
        const auto rune_yaw = x(4), blade_angle = x(5);

        const auto R_mark_ypd = Eigen::Vector3d(R_mark_yaw, R_mark_pitch, R_mark_distance);
        const auto J_R_mark_xyz_vs_R_mark_ypd = util::ypd2xyz_jacobian(R_mark_ypd);

        // B = R + [r*sin(ry)*sin(a), -r*cos(ry)*sin(a), r*cos(a)]^T
        // => ∂B/∂ry = [r*cos(ry)*sin(a),  r*sin(ry)*sin(a), 0]^T
        // => ∂B/∂a  = [r*sin(ry)*cos(a), -r*cos(ry)*cos(a), -r*sin(a)]^T
        const auto r_blade         = kRuneRadius;
        const auto cos_rune_yaw    = std::cos(rune_yaw);
        const auto sin_rune_yaw    = std::sin(rune_yaw);
        const auto cos_blade_angle = std::cos(blade_angle);
        const auto sin_blade_angle = std::sin(blade_angle);

        auto J_blade_offset_vs_rune_yaw = Eigen::Vector3d {};
        J_blade_offset_vs_rune_yaw << r_blade * cos_rune_yaw * sin_blade_angle,
            r_blade * sin_rune_yaw * sin_blade_angle, 0;

        auto J_blade_offset_vs_blade_angle = Eigen::Vector3d {};
        J_blade_offset_vs_blade_angle << r_blade * sin_rune_yaw * cos_blade_angle,
            -r_blade * cos_rune_yaw * cos_blade_angle, -r_blade * sin_blade_angle;

        Eigen::Matrix<double, 3, 6> J_blade_xyz_vs_x = Eigen::Matrix<double, 3, 6>::Zero();
        J_blade_xyz_vs_x.col(0)                      = J_R_mark_xyz_vs_R_mark_ypd.col(0);
        J_blade_xyz_vs_x.col(2)                      = J_R_mark_xyz_vs_R_mark_ypd.col(1);
        J_blade_xyz_vs_x.col(3)                      = J_R_mark_xyz_vs_R_mark_ypd.col(2);
        J_blade_xyz_vs_x.col(4)                      = J_blade_offset_vs_rune_yaw;
        J_blade_xyz_vs_x.col(5)                      = J_blade_offset_vs_blade_angle;

        const auto R_mark_xyz = util::ypd2xyz(R_mark_ypd);
        const auto blade_xyz =
            Eigen::Vector3d { R_mark_xyz(0) + r_blade * sin_rune_yaw * sin_blade_angle,
                R_mark_xyz(1) - r_blade * cos_rune_yaw * sin_blade_angle,
                R_mark_xyz(2) + r_blade * cos_blade_angle };

        const auto J_blade_ypd_vs_blade_xyz = util::xyz2ypd_jacobian(blade_xyz);
        // H_B = ∂z_B/∂x = (∂z_B/∂B) * (∂B/∂x)
        H.block<3, 6>(4, 0) = J_blade_ypd_vs_blade_xyz * J_blade_xyz_vs_x;

        return H;
    }

    static auto shared_h(ObservationSubstate const& x) -> SmallEKF::ZVec {
        auto z = SmallEKF::ZVec {};

        // Linear observation:
        // z0 = x0, z1 = x2, z2 = x3, z3 = x5.
        z(0) = x(0);
        z(1) = x(2);
        z(2) = x(3);
        z(3) = x(5);

        // Nonlinear part:
        // blade_xyz = R_mark_xyz + [r*sin(ry)*sin(a), -r*cos(ry)*sin(a), r*cos(a)]^T
        // z4..z6 = xyz2ypd(blade_xyz)
        const auto R_mark_ypd = Eigen::Vector3d(x(0), x(2), x(3));
        const auto R_mark_xyz = util::ypd2xyz(R_mark_ypd);

        const auto rune_yaw    = x(4);
        const auto blade_angle = x(5);

        const auto cos_rune_yaw    = std::cos(rune_yaw);
        const auto sin_rune_yaw    = std::sin(rune_yaw);
        const auto cos_blade_angle = std::cos(blade_angle);
        const auto sin_blade_angle = std::sin(blade_angle);

        const auto blade_xyz =
            Eigen::Vector3d { R_mark_xyz(0) + kRuneRadius * sin_rune_yaw * sin_blade_angle,
                R_mark_xyz(1) - kRuneRadius * cos_rune_yaw * sin_blade_angle,
                R_mark_xyz(2) + kRuneRadius * cos_blade_angle };
        const auto blade_ypd = util::xyz2ypd(blade_xyz);

        z(4) = blade_ypd(0);
        z(5) = blade_ypd(1);
        z(6) = blade_ypd(2);
        return z;
    }

    static auto small_H(SmallEKF::XVec const& x) -> SmallEKF::HMat {
        SmallEKF::HMat H    = SmallEKF::HMat::Zero();
        const auto x_shared = observation_substate(x);
        H.block<7, 6>(0, 0) = shared_H(x_shared);
        return H;
    }

    static auto small_h(SmallEKF::XVec const& x) -> SmallEKF::ZVec {
        return shared_h(observation_substate(x));
    }

    // Large EKF state :
    // x_l = [R_yaw, R_yaw_w, R_pitch, R_dist, rune_yaw, blade_phase, amp, omega, phi].
    // blade_w(t) = s * [amp * sin(phi(t)) + (B - amp)],  phi_dot = omega.
    static auto large_x(FanBlade3D const& blade, int clockwise) -> LargeEKF::XVec {
        const auto [quat_x, quat_y, quat_z, quat_w] = blade.orientation;
        const auto orientation = Eigen::Quaterniond { quat_w, quat_x, quat_y, quat_z };

        auto R_mark_xyz = Eigen::Vector3d {};
        blade.R_mark_position.copy_to(R_mark_xyz);

        const auto R_mark_ypd      = util::xyz2ypd(R_mark_xyz);
        const auto R_mark_yaw      = R_mark_ypd[0];
        const auto R_mark_pitch    = R_mark_ypd[1];
        const auto R_mark_distance = R_mark_ypd[2];

        const auto rune_ypr  = util::eulers(orientation);
        const auto rune_yaw  = rune_ypr[0];
        const auto rune_roll = rune_ypr[2];

        const auto amplitude_mean = 0.5 * (kLargeRuneAmplitudeMin + kLargeRuneAmplitudeMax);
        const auto omega_mean     = 0.5 * (kLargeRuneOmegaMin + kLargeRuneOmegaMax);
        (void)clockwise;

        const auto x = LargeEKF::XVec { R_mark_yaw, 0., R_mark_pitch, R_mark_distance, rune_yaw,
            rune_roll, amplitude_mean, omega_mean, 0. };
        return x;
    }

    static auto large_F(double dt, LargeEKF::XVec const& x, int clockwise) -> LargeEKF::AMat {
        LargeEKF::AMat F = LargeEKF::AMat::Identity();
        // R_mark_yaw_{k+1} = R_mark_yaw_{k} + R_mark_angular_velocity_{k} * dt
        F(0, 1)              = dt;
        const auto amp_raw   = x(6);
        const auto omega_raw = x(7);
        const auto amp       = std::clamp(amp_raw, kLargeRuneAmplitudeMin, kLargeRuneAmplitudeMax);
        const auto omega     = std::clamp(omega_raw, kLargeRuneOmegaMin, kLargeRuneOmegaMax);
        const auto phi       = x(8);
        const auto next_phi  = phi + omega * dt;
        const auto direction = clockwise >= 0 ? 1.0 : -1.0;

        // amp_{k+1} = clamp(amp_k), omega_{k+1} = clamp(omega_k), Jacobian uses straight-through
        // derivative (1.0) to preserve covariance propagation.
        F(6, 6) = 1.0;
        F(7, 7) = 1.0;
        // phi_{k+1} = phi_{k} + omega_{k} * dt, omega may be saturated by clamp.
        F(8, 7) = dt;

        // blade_angle_{k+1} = blade_angle_{k} + direction * integral(speed, dt)
        // speed(t) = amp * sin(omega * t + phi) + (kLargeRuneOffsetBase - amp)
        // α_{k+1} = α_k + s * I(A, w, φ), 其中 s∈{+1,-1}
        // A = clamp(A_raw), w = clamp(w_raw), φ' = φ + w*dt
        // I(A,w,φ)= A/w * (cosφ - cosφ') + (B-A)dt  (w ≠ 0)
        // ∂α/∂A_raw = (∂α/∂A) * (dA/dA_raw), ∂α/∂w_raw = (∂α/∂w) * (dw/dw_raw)
        if (std::abs(omega) > 1e-6) {
            const auto cos_delta = std::cos(phi) - std::cos(next_phi);
            // ∂α/∂A_raw = s * (cosφ - cosφ')/w - s*dt, with straight-through dA/dA_raw = 1
            F(5, 6) = direction * (cos_delta / omega - dt);
            // ∂α/∂w_raw = s * A * [ sin(φ')*dt/w - (cosφ-cosφ')/w^2 ], with dω/dω_raw = 1
            F(5, 7) =
                direction * amp * (std::sin(next_phi) * dt / omega - cos_delta / (omega * omega));
            // ∂α/∂φ = s * A * (sinφ' - sinφ)/w
            F(5, 8) = direction * amp * (std::sin(next_phi) - std::sin(phi)) / omega;
        } else {
            // omega ~ 0: use the same constant-speed approximation branch as large_f.
            F(5, 6) = direction * dt * (std::sin(phi) - 1.0);
            F(5, 7) = 0.0;
            F(5, 8) = direction * dt * amp * std::cos(phi);
        }

        return F;
    }

    static auto large_f(double dt, LargeEKF::XVec const& x, int clockwise) -> LargeEKF::XVec {
        auto x_prior = x;

        const auto R_mark_angular_velocity = x(1);
        const auto amp       = std::clamp(x(6), kLargeRuneAmplitudeMin, kLargeRuneAmplitudeMax);
        const auto omega     = std::clamp(x(7), kLargeRuneOmegaMin, kLargeRuneOmegaMax);
        const auto phi       = x(8);
        const auto direction = clockwise >= 0 ? 1.0 : -1.0;

        // R_yaw' = R_yaw + R_yaw_w * dt
        x_prior(0) += R_mark_angular_velocity * dt;

        // α_{k+1} = α_k + s * ∫[A*sin(ωt+φ) + (B-A)]dt, t∈[0,dt]
        const auto next_phi = phi + omega * dt;
        if (std::abs(omega) > 1e-6) {
            // α_{k+1} = α_k + s * [ A/w*(cosφ - cos(φ+w*dt)) + (B-A)dt ]
            const auto delta_angle = (amp / omega) * (std::cos(phi) - std::cos(next_phi))
                + (kLargeRuneOffsetBase - amp) * dt;
            x_prior(5) += delta_angle * direction;
        } else {
            // w≈0 时退化为常速: α_{k+1} = α_k + s * v(0) * dt
            const auto current_speed = amp * std::sin(phi) + (kLargeRuneOffsetBase - amp);
            x_prior(5) += current_speed * dt * direction;
        }

        // phi' = phi + omega * dt
        x_prior(8) = next_phi;

        // amp/omega are constrained to physically valid ranges.
        x_prior(6) = amp;
        x_prior(7) = omega;

        // Wrap periodic states to [-pi, pi].
        x_prior(0) = util::normalize_angle(x_prior(0));
        x_prior(2) = util::normalize_angle(x_prior(2));
        x_prior(4) = util::normalize_angle(x_prior(4));
        x_prior(5) = util::normalize_angle(x_prior(5));
        x_prior(8) = util::normalize_angle(x_prior(8));
        clamp_large_shape_states(x_prior);

        return x_prior;
    }

    // Process noise covariance matrix Q
    static auto large_Q(double dt) -> LargeEKF::QMat {
        // R_mark_yaw, R_mark_angular_velocity, R_mark_pitch, R_mark_distance, rune_yaw,
        // blade_angle, amp, omega, phi
        const auto R_mark_yaw_acc_var = 0.9; // R标偏航角加速度方差 (v1)
        const auto amp_rw_var         = 1e-3;
        const auto omega_rw_var       = 1e-3;
        const auto a1                 = dt * dt * dt * dt / 4.;
        const auto b1                 = dt * dt * dt / 2.;
        const auto c1                 = dt * dt;
        const auto q_amp              = amp_rw_var * dt;
        const auto q_omega            = omega_rw_var * dt;

        LargeEKF::QMat Q = LargeEKF::QMat::Zero();
        Q(0, 0)          = a1 * R_mark_yaw_acc_var;
        Q(0, 1)          = b1 * R_mark_yaw_acc_var;
        Q(1, 0)          = b1 * R_mark_yaw_acc_var;
        Q(1, 1)          = c1 * R_mark_yaw_acc_var;
        Q(5, 5)          = 0.09;    // blade_angle
        Q(6, 6)          = q_amp;   // amp
        Q(7, 7)          = q_omega; // omega
        Q(8, 8)          = 1.0;     // phi
        return Q;
    }

    static auto large_P() -> LargeEKF::PMat {
        LargeEKF::PMat P = LargeEKF::PMat::Identity();
        // R_mark_yaw, R_mark_angular_velocity, R_mark_pitch, R_mark_distance, rune_yaw,
        // blade_angle, amp, omega, phi
        P.diagonal() << 10., 10., 10., 10., 10., 10., 10., 10., 400.;
        return P;
    }

    static auto large_H(LargeEKF::XVec const& x) -> LargeEKF::HMat {
        LargeEKF::HMat H    = LargeEKF::HMat::Zero();
        const auto x_shared = observation_substate(x);
        H.block<7, 6>(0, 0) = shared_H(x_shared);
        return H;
    }

    static auto large_h(LargeEKF::XVec const& x) -> LargeEKF::ZVec {
        return shared_h(observation_substate(x));
    }

public:
    template <typename T>
    static auto rune_x(FanBlade3D const& blade, int clockwise) -> typename T::XVec {
        if constexpr (is_small_ekf_v<T>) {
            return small_x(blade, clockwise);
        } else {
            return large_x(blade, clockwise);
        }
    }

    template <typename T>
    static auto rune_F(double dt, typename T::XVec const& x, int clockwise) -> typename T::AMat {
        if constexpr (is_small_ekf_v<T>) {
            return small_F(dt);
        } else {
            return large_F(dt, x, clockwise);
        }
    }

    template <typename T>
    static auto rune_f(double dt, int clockwise) {
        return [dt, clockwise](typename T::XVec const& x) {
            if constexpr (is_small_ekf_v<T>) {
                return small_f(dt, x);
            } else {
                return large_f(dt, x, clockwise);
            }
        };
    }

    template <typename T>
    static auto rune_P() -> typename T::PMat {
        if constexpr (is_small_ekf_v<T>) {
            return small_P();
        } else {
            return large_P();
        }
    }

    template <typename T>
    static auto rune_Q(double dt) -> typename T::QMat {
        if constexpr (is_small_ekf_v<T>) {
            return small_Q(dt);
        } else {
            return large_Q(dt);
        }
    }

    template <typename T>
    static auto rune_H(typename T::XVec const& x) -> typename T::HMat {
        if constexpr (is_small_ekf_v<T>) {
            return small_H(x);
        } else {
            return large_H(x);
        }
    }

    template <typename T>
    static auto rune_h(typename T::XVec const& x) -> typename T::ZVec {
        if constexpr (is_small_ekf_v<T>) {
            return small_h(x);
        } else {
            return large_h(x);
        }
    }

    template <typename T>
    static auto rune_R() -> typename T::RMat {
        typename T::RMat R = T::RMat::Zero();
        // 线性观测噪声
        R(0, 0) = 0.01; // R_yaw
        R(1, 1) = 0.01; // R_pitch
        R(2, 2) = 0.5;  // R_dis
        R(3, 3) = 0.1;  // blade_angle (roll)

        // 非线性观测噪声
        R(4, 4) = 0.01; // B_yaw
        R(5, 5) = 0.01; // B_pitch
        R(6, 6) = 0.5;  // B_dis

        return R;
    }

    template <typename T>
    static auto rune_z_subtract(typename T::ZVec const& a, typename T::ZVec const& b) ->
        typename T::ZVec {
        // Innovation on SE(2)-like angle components:
        // y = a ⊖ b,  y_i = wrap(a_i - b_i) for angle indices.
        typename T::ZVec result = a - b;
        result(0)               = util::normalize_angle(result(0));
        result(1)               = util::normalize_angle(result(1));
        result(3)               = util::normalize_angle(result(3));
        result(4)               = util::normalize_angle(result(4));
        result(5)               = util::normalize_angle(result(5));
        return result;
    }

    template <typename T>
    static auto rune_x_add(typename T::XVec const& a, typename T::XVec const& b) ->
        typename T::XVec {
        typename T::XVec result = a + b;
        result(0)               = util::normalize_angle(result(0));
        result(2)               = util::normalize_angle(result(2));
        result(4)               = util::normalize_angle(result(4));
        result(5)               = util::normalize_angle(result(5));
        if constexpr (is_large_ekf_v<T>) {
            result(8) = util::normalize_angle(result(8));
            clamp_large_shape_states(result);
        }
        return result;
    }
};

} // namespace rmcs::predictor
