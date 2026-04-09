#include "outpost_robot_state.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

#include "utility/math/angle.hpp"
#include "utility/time.hpp"

using namespace rmcs::predictor;

namespace {
struct OutpostObservation {
    OutpostRobotState::EKF::ZVec z;
    OutpostRobotState::EKF::RMat R;
    Eigen::Vector3d xyz;
    Eigen::Vector3d ypr;
    Eigen::Vector3d ypd;
};

auto apply_constraints(OutpostRobotState::EKF::XVec& x) -> void {
    x[5] = rmcs::util::normalize_angle(x[5]);
}

auto outpost_observation(rmcs::Armor3D const& armor) -> OutpostObservation {
    auto const [pos_x, pos_y, pos_z] = armor.translation;
    auto const xyz                   = Eigen::Vector3d { pos_x, pos_y, pos_z };

    auto const [quat_x, quat_y, quat_z, quat_w] = armor.orientation;
    auto const orientation = Eigen::Quaterniond { quat_w, quat_x, quat_y, quat_z };

    auto const ypr = rmcs::util::eulers(orientation);
    auto const ypd = rmcs::util::xyz2ypd(xyz);

    auto z = OutpostRobotState::EKF::ZVec {};
    z << ypd[0], ypd[1], ypd[2], ypr[0];

    return { z, OutpostEKFParameters::R(xyz, ypr, ypd), xyz, ypr, ypd };
}

auto slot_sorted_observation_indices(OutpostState::FrameMatch const& frame_match)
    -> std::vector<std::size_t> {
    auto observation_order = std::vector<std::size_t>(frame_match.slots_by_observation.size());
    std::iota(observation_order.begin(), observation_order.end(), std::size_t { 0 });
    std::sort(
        observation_order.begin(), observation_order.end(), [&](std::size_t lhs, std::size_t rhs) {
            return frame_match.slots_by_observation[lhs] < frame_match.slots_by_observation[rhs];
        });
    return observation_order;
}

auto apply_observation_update(OutpostRobotState::EKF& ekf, OutpostObservation const& observation,
    int slot_idx, int order_idx) -> void {
    ekf.update(
        observation.z,
        [slot_idx, order_idx](OutpostRobotState::EKF::XVec const& x) {
            return OutpostEKFParameters::h(x, slot_idx, order_idx);
        },
        [slot_idx, order_idx](OutpostRobotState::EKF::XVec const& x) {
            return OutpostEKFParameters::H(x, slot_idx, order_idx);
        },
        observation.R, OutpostEKFParameters::x_add, OutpostEKFParameters::z_subtract);

    apply_constraints(ekf.x);
}

auto apply_frame_observation_updates(OutpostRobotState::EKF& ekf,
    std::span<rmcs::Armor3D const> armors, OutpostState::FrameMatch const& frame_match) -> void {
    for (auto obs_index : slot_sorted_observation_indices(frame_match)) {
        auto observation = outpost_observation(armors[obs_index]);
        auto slot_idx    = frame_match.slots_by_observation[obs_index];
        apply_observation_update(ekf, observation, slot_idx, frame_match.order_idx);
    }
}

auto covariance_is_converged(OutpostRobotState::EKF::PMat const& P) -> bool {
    auto const xy_var_ok  = P(0, 0) < 0.1 && P(2, 2) < 0.1;
    auto const z_var_ok   = P(4, 4) < 0.05;
    auto const yaw_var_ok = P(5, 5) < 0.2;
    return xy_var_ok && z_var_ok && yaw_var_ok;
}

auto build_hypothesis_state(OutpostRobotState::EKF::XVec const& fallback_x,
    OutpostRobotState::EKF::XVec const& pre_predict_x, double predict_dt_s,
    bool has_predict_context, int spin_sign) -> OutpostRobotState::EKF::XVec {
    auto x_test = has_predict_context ? pre_predict_x : fallback_x;
    if (has_predict_context && predict_dt_s > 0.0) {
        x_test = OutpostEKFParameters::f(predict_dt_s, spin_sign)(x_test);
    } else {
        apply_constraints(x_test);
    }
    return x_test;
}

auto build_match_context(OutpostRobotState::EKF const& ekf,
    OutpostRobotState::EKF::XVec const& pre_predict_x, double predict_dt_s,
    bool has_predict_context) -> OutpostState::MatchContext {
    return {
        ekf.x,
        ekf.P(),
        build_hypothesis_state(ekf.x, pre_predict_x, predict_dt_s, has_predict_context, -1),
        build_hypothesis_state(ekf.x, pre_predict_x, predict_dt_s, has_predict_context, +1),
    };
}
} // namespace

OutpostRobotState::OutpostRobotState(Clock::time_point stamp) noexcept
    : time_stamp { stamp } { }

auto OutpostRobotState::initialize(Armor3D const& armor, Clock::time_point t) -> void {
    if (armor.genre != DeviceId::OUTPOST) {
        initialized = false;
        outpost_state.reset();
        return;
    }

    color      = armor_color2camp_color(armor.color);
    time_stamp = t;
    ekf =
        EKF { OutpostEKFParameters::x(armor), OutpostEKFParameters::P_initial_dig().asDiagonal() };
    apply_constraints(ekf.x);
    pre_predict_x_       = EKF::XVec::Zero();
    last_predict_dt_s_   = 0.0;
    has_predict_context_ = false;
    update_count_        = 0;
    outpost_state.reset();
    initialized = true;
}

auto OutpostRobotState::predict(Clock::time_point t) -> void {
    if (initialized) {
        auto dt = util::delta_time(t, time_stamp);
        if (dt > reset_interval) {
            initialized          = false;
            time_stamp           = t;
            has_predict_context_ = false;
            update_count_        = 0;
            outpost_state.reset();
            return;
        }

        auto dt_s      = dt.count();
        auto spin_sign = outpost_state.current_spin_sign();
        pre_predict_x_       = ekf.x;
        last_predict_dt_s_   = dt_s;
        has_predict_context_ = true;
        ekf.predict(
            OutpostEKFParameters::f(dt_s, spin_sign),
            [dt_s](EKF::XVec const&) { return OutpostEKFParameters::F(dt_s); },
            OutpostEKFParameters::Q(dt_s));
        apply_constraints(ekf.x);
    }

    time_stamp = t;
}

auto OutpostRobotState::match(Armor3D const& armor) const -> MatchResult {
    if (!initialized || armor.genre != DeviceId::OUTPOST) return { -1, 1e10, false };

    auto match_result = outpost_state.match_armor(
        armor, build_match_context(ekf, pre_predict_x_, last_predict_dt_s_, has_predict_context_));
    return { match_result.armor_id, match_result.error, match_result.is_valid };
}

auto OutpostRobotState::update(Armor3D const& armor) -> bool {
    if (armor.genre != DeviceId::OUTPOST) return false;
    return update(std::span<Armor3D const> { &armor, static_cast<std::size_t>(1) });
}

auto OutpostRobotState::update(std::span<Armor3D const> armors) -> bool {
    if (armors.empty()) return false;

    auto used_armors = armors.first(
        std::min<std::size_t>(armors.size(), OutpostEKFParameters::kOutpostArmorCount));
    if (used_armors.front().genre != DeviceId::OUTPOST) return false;

    if (!initialized) initialize(used_armors.front(), time_stamp);
    auto frame_match = outpost_state.match_frame(
        used_armors, build_match_context(ekf, pre_predict_x_, last_predict_dt_s_, has_predict_context_));
    if (!frame_match.valid) return false;

    ekf.x = build_hypothesis_state(
        ekf.x, pre_predict_x_, last_predict_dt_s_, has_predict_context_, frame_match.spin_sign);
    apply_constraints(ekf.x);
    apply_frame_observation_updates(ekf, used_armors, frame_match);
    outpost_state.apply_frame_match(frame_match);
    update_count_++;
    return true;
}

auto OutpostRobotState::is_converged() const -> bool {
    return initialized && outpost_state.spin_locked() && covariance_is_converged(ekf.P())
        && update_count_ > 10;
}

auto OutpostRobotState::get_snapshot() const -> Snapshot {
    auto outpost_order_idx = outpost_state.current_order_idx();
    auto outpost_spin_sign = outpost_state.current_spin_sign();
    return { ekf.x, color, armor_num, time_stamp, outpost_spin_sign, outpost_order_idx };
}

auto OutpostRobotState::distance() const -> double {
    return std::sqrt(ekf.x[0] * ekf.x[0] + ekf.x[2] * ekf.x[2]);
}
