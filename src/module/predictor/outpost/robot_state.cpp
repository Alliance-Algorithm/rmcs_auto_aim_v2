#include "robot_state.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <span>

#include "module/predictor/outpost/snapshot.hpp"
#include "utility/math/angle.hpp"
#include "utility/math/mahalanobis.hpp"
#include "utility/time.hpp"

using namespace rmcs::predictor;

namespace {

constexpr int kUnknownArmorId = -1;
using OutpostEKF              = OutpostRobotState::EKF;

auto normalize_spin_sign(int spin_sign) -> int {
    if (spin_sign > 0) return +1;
    if (spin_sign < 0) return -1;
    return 0;
}

struct OutpostObservation {
    OutpostEKF::ZVec z;
    OutpostEKF::RMat R;
    Eigen::Vector3d xyz;
    Eigen::Vector3d ypr;
    Eigen::Vector3d ypd;
};

auto make_observation(rmcs::Armor3D const& armor) -> OutpostObservation {
    auto const [pos_x, pos_y, pos_z] = armor.translation;
    auto const xyz                   = Eigen::Vector3d { pos_x, pos_y, pos_z };

    auto const [quat_x, quat_y, quat_z, quat_w] = armor.orientation;
    auto const orientation = Eigen::Quaterniond { quat_w, quat_x, quat_y, quat_z };

    auto const ypr = rmcs::util::eulers(orientation);
    auto const ypd = rmcs::util::xyz2ypd(xyz);

    auto z = OutpostEKF::ZVec { };
    z << ypd[0], ypd[1], ypd[2], ypr[0];

    return { z, OutpostEKFParameters::R(xyz, ypr, ypd), xyz, ypr, ypd };
}

enum class SwitchEvent {
    Stay,
    ClockwiseSwitch,
    CounterClockwiseSwitch,
    Invalid,
};

struct AssociationDecision {
    int armor_id { kUnknownArmorId };
    double error { std::numeric_limits<double>::infinity() };
    bool is_valid { false };
    SwitchEvent event { SwitchEvent::Invalid };
    int inferred_spin_sign { 0 };
    double phase_offset { 0.0 };
    double height_offset { 0.0 };
    bool extends_layout { false };
};

struct BestMatch {
    OutpostObservation observation;
    AssociationDecision decision;
};

struct MatchingConfig {
    double azimuth_gate { rmcs::util::deg2rad(35.0) };
    double z_gate { 0.05 };
    double slot_phase_tolerance { rmcs::util::deg2rad(8.0) };
    double slot_height_tolerance { 0.02 };
    double mahalanobis_gate { 25.0 };
    double layout_extension_penalty { 0.25 };
    double continuity_bonus { 0.10 };
};

struct TrackingConfig {
    std::chrono::duration<double> reset_interval { 1.5 };
    int spin_confirm_switches { 2 };
    int min_converged_updates { 6 };
    MatchingConfig matching { };
};

struct SpinTracker {
    int locked_sign { 0 };
    int candidate_sign { 0 };
    int candidate_count { 0 };
    bool locked { false };

    auto reset() -> void { *this = { }; }

    auto current_sign() const -> int {
        if (locked) return locked_sign;
        return candidate_sign;
    }

    auto observe_switch(int inferred_spin_sign, int confirm_switches) -> void {
        auto const normalized = normalize_spin_sign(inferred_spin_sign);
        if (normalized == 0 || locked) return;

        if (candidate_sign == normalized) {
            candidate_count++;
        } else {
            candidate_sign  = normalized;
            candidate_count = 1;
        }

        if (candidate_count < confirm_switches) return;

        locked_sign = candidate_sign;
        locked      = true;
    }
};

auto assigned_count(OutpostArmorLayout const& layout) -> int {
    return static_cast<int>(std::ranges::count(layout.slots, true, &OutpostArmorSlot::assigned));
}

auto has_assigned_slot(OutpostArmorLayout const& layout, int armor_id) -> bool {
    return armor_id >= 0 && armor_id < OutpostEKFParameters::kOutpostArmorCount
        && layout.slots[armor_id].assigned;
}

auto first_unassigned_slot(OutpostArmorLayout const& layout) -> int {
    for (int id = 0; id < OutpostEKFParameters::kOutpostArmorCount; ++id) {
        if (!layout.slots[id].assigned) return id;
    }
    return kUnknownArmorId;
}

auto layout_after_association(OutpostArmorLayout const& layout, AssociationDecision const& decision)
    -> OutpostArmorLayout {
    auto next_layout = layout;
    if (!decision.extends_layout) return next_layout;

    next_layout.slots[decision.armor_id].phase_offset  = decision.phase_offset;
    next_layout.slots[decision.armor_id].height_offset = decision.height_offset;
    next_layout.slots[decision.armor_id].assigned      = true;
    return next_layout;
}

auto switch_phase_delta(int inferred_spin_sign) -> double {
    return -normalize_spin_sign(inferred_spin_sign) * OutpostEKFParameters::kPhaseStep;
}

auto switch_height_deltas(int inferred_spin_sign) -> std::array<double, 2> {
    if (normalize_spin_sign(inferred_spin_sign) > 0) {
        return { rmcs::kOutpostArmorHeightStep, -2.0 * rmcs::kOutpostArmorHeightStep };
    }
    return { -rmcs::kOutpostArmorHeightStep, 2.0 * rmcs::kOutpostArmorHeightStep };
}

auto consider_candidate(AssociationDecision const& candidate, AssociationDecision& best_decision)
    -> void {
    if (!candidate.is_valid || candidate.error >= best_decision.error) return;
    best_decision = candidate;
}

class AssociationEngine {
public:
    AssociationEngine(OutpostEKF::XVec const& x, OutpostEKF::PMat const& P,
        OutpostArmorLayout const& layout, int current_armor_id, SpinTracker const& spin,
        MatchingConfig const& config) noexcept
        : x_ { x }
        , P_ { P }
        , layout_ { layout }
        , current_armor_id_ { current_armor_id }
        , spin_ { spin }
        , config_ { config } { }

    auto decide(OutpostObservation const& observation) const -> AssociationDecision {
        if (!has_assigned_slot(layout_, current_armor_id_)) return { };

        auto best_decision        = AssociationDecision { };
        auto const current_phase  = layout_.slots[current_armor_id_].phase_offset;
        auto const current_height = layout_.slots[current_armor_id_].height_offset;

        consider_candidate(evaluate_candidate(observation, current_armor_id_, current_phase,
                               current_height, SwitchEvent::Stay, 0, false),
            best_decision);

        if (spin_.locked) {
            auto const event = spin_.locked_sign > 0 ? SwitchEvent::CounterClockwiseSwitch
                                                     : SwitchEvent::ClockwiseSwitch;
            consider_switch_direction(observation, current_phase, current_height, spin_.locked_sign,
                event, best_decision);
        } else {
            consider_switch_direction(observation, current_phase, current_height, -1,
                SwitchEvent::ClockwiseSwitch, best_decision);
            consider_switch_direction(observation, current_phase, current_height, +1,
                SwitchEvent::CounterClockwiseSwitch, best_decision);
        }

        return best_decision;
    }

private:
    auto evaluate_candidate(OutpostObservation const& observation, int armor_id,
        double phase_offset, double height_offset, SwitchEvent event, int inferred_spin_sign,
        bool extends_layout) const -> AssociationDecision {
        auto const predicted_xyz =
            OutpostEKFParameters::h_armor_xyz(x_, phase_offset, height_offset);
        auto const predicted_ypd = rmcs::util::xyz2ypd(predicted_xyz);

        auto const azimuth_error =
            std::abs(rmcs::util::normalize_angle(observation.ypd[0] - predicted_ypd[0]));
        auto const z_error = std::abs(observation.xyz[2] - predicted_xyz[2]);

        // 这里没有加yaw约束，一是因为yaw的抖动太大，二是因为大部分图像中 一帧只有一块装甲板
        if (azimuth_error > config_.azimuth_gate || z_error > config_.z_gate) {
            return { };
        }

        auto const H           = OutpostEKFParameters::H(x_, phase_offset, height_offset);
        auto const z_hat       = OutpostEKFParameters::h(x_, phase_offset, height_offset);
        auto const innovation  = OutpostEKFParameters::z_subtract(observation.z, z_hat);
        auto const S           = H * P_ * H.transpose() + observation.R;
        auto const mahalanobis = rmcs::util::mahalanobis_distance(innovation, S);
        if (!mahalanobis.has_value() || *mahalanobis > config_.mahalanobis_gate) {
            return { };
        }

        auto error = *mahalanobis;
        if (extends_layout) error += config_.layout_extension_penalty;
        if (event == SwitchEvent::Stay) error -= config_.continuity_bonus;

        return { armor_id, error, true, event, normalize_spin_sign(inferred_spin_sign),
            rmcs::util::normalize_angle(phase_offset), height_offset, extends_layout };
    }

    auto find_matching_slot(double phase_offset, double height_offset, int excluded_armor_id) const
        -> int {
        auto best_slot     = kUnknownArmorId;
        auto best_mismatch = std::numeric_limits<double>::infinity();

        for (int id = 0; id < OutpostEKFParameters::kOutpostArmorCount; ++id) {
            if (!layout_.slots[id].assigned || id == excluded_armor_id) continue;

            auto const phase_error = std::abs(
                rmcs::util::normalize_angle(layout_.slots[id].phase_offset - phase_offset));
            auto const height_error = std::abs(layout_.slots[id].height_offset - height_offset);
            if (phase_error > config_.slot_phase_tolerance
                || height_error > config_.slot_height_tolerance)
                continue;

            auto const mismatch = phase_error + height_error / rmcs::kOutpostArmorHeightStep;
            if (mismatch >= best_mismatch) continue;

            best_mismatch = mismatch;
            best_slot     = id;
        }

        return best_slot;
    }

    auto consider_switch_direction(OutpostObservation const& observation, double current_phase,
        double current_height, int inferred_spin_sign, SwitchEvent event,
        AssociationDecision& best_decision) const -> void {
        auto const candidate_phase =
            rmcs::util::normalize_angle(current_phase + switch_phase_delta(inferred_spin_sign));

        for (auto const height_delta : switch_height_deltas(inferred_spin_sign)) {
            auto const candidate_height = current_height + height_delta;

            auto armor_id =
                find_matching_slot(candidate_phase, candidate_height, current_armor_id_);
            auto extends_layout = false;
            if (armor_id == kUnknownArmorId) {
                armor_id       = first_unassigned_slot(layout_);
                extends_layout = (armor_id != kUnknownArmorId);
            }
            if (armor_id == kUnknownArmorId) continue;

            consider_candidate(evaluate_candidate(observation, armor_id, candidate_phase,
                                   candidate_height, event, inferred_spin_sign, extends_layout),
                best_decision);
        }
    }

    OutpostEKF::XVec const& x_;
    OutpostEKF::PMat const& P_;
    OutpostArmorLayout const& layout_;
    int current_armor_id_ { kUnknownArmorId };
    SpinTracker const& spin_;
    MatchingConfig const& config_;
};

} // namespace

struct OutpostRobotState::Impl {
    explicit Impl(TimePoint stamp) noexcept
        : time_stamp { stamp } { }

    auto initialize(Armor3D const& armor, TimePoint t) -> void {
        color      = armor_color2camp_color(armor.color);
        ekf        = EKF { OutpostEKFParameters::x(armor),
                   OutpostEKFParameters::P_initial_dig().asDiagonal() };
        time_stamp = t;

        layout                   = OutpostArmorLayout { };
        layout.slots[0].assigned = true;

        spin.reset();
        update_count     = 0;
        current_armor_id = 0;
        initialized      = true;
    }

    auto predict(TimePoint t) -> void {
        if (initialized) {
            auto dt = rmcs::util::delta_time(t, time_stamp);
            if (dt > config.reset_interval) {
                reset_runtime_state(t);
                return;
            }

            auto const dt_s = dt.count();
            ekf.predict(
                OutpostEKFParameters::f(dt_s, spin.current_sign()),
                [dt_s](EKF::XVec const&) { return OutpostEKFParameters::F(dt_s); },
                OutpostEKFParameters::Q(dt_s));
        }

        time_stamp = t;
    }

    auto update(std::span<Armor3D const> armors) -> bool {
        if (armors.empty()) return false;

        if (!initialized) {
            initialize(armors.front(), time_stamp);
            return true;
        }

        auto best_match = select_best_match(armors);
        if (!best_match.has_value()) return false;

        apply_association(best_match->decision, best_match->observation);
        return true;
    }

    auto is_converged() const -> bool {
        return initialized && spin.locked
            && assigned_count(layout) == OutpostEKFParameters::kOutpostArmorCount
            && update_count >= config.min_converged_updates;
    }

    auto get_snapshot() const -> Snapshot {
        return detail::make_outpost_snapshot(
            ekf.x, color, assigned_count(layout), time_stamp, spin.current_sign(), layout);
    }

    auto distance() const -> double { return std::sqrt(ekf.x[0] * ekf.x[0] + ekf.x[2] * ekf.x[2]); }

private:
    auto reset_runtime_state(TimePoint t) -> void {
        color            = CampColor::UNKNOWN;
        ekf              = EKF { };
        layout           = OutpostArmorLayout { };
        time_stamp       = t;
        initialized      = false;
        current_armor_id = kUnknownArmorId;
        spin.reset();
        update_count = 0;
    }

    auto select_best_match(std::span<Armor3D const> armors) const -> std::optional<BestMatch> {
        auto best_match = std::optional<BestMatch> { };
        auto matcher =
            AssociationEngine { ekf.x, ekf.P(), layout, current_armor_id, spin, config.matching };

        for (auto const& armor : armors) {
            auto observation = make_observation(armor);
            auto decision    = matcher.decide(observation);
            if (!decision.is_valid) continue;
            if (best_match.has_value() && decision.error >= best_match->decision.error) continue;

            best_match = BestMatch { observation, decision };
        }

        return best_match;
    }

    auto apply_association(
        AssociationDecision const& decision, OutpostObservation const& observation) -> void {
        auto const next_layout = layout_after_association(layout, decision);

        ekf.update(
            observation.z,
            [layout = next_layout, armor_id = decision.armor_id](
                EKF::XVec const& x) { return OutpostEKFParameters::h(x, layout, armor_id); },
            [layout = next_layout, armor_id = decision.armor_id](
                EKF::XVec const& x) { return OutpostEKFParameters::H(x, layout, armor_id); },
            observation.R, OutpostEKFParameters::x_add, OutpostEKFParameters::z_subtract);

        layout           = next_layout;
        current_armor_id = decision.armor_id;
        if (decision.event != SwitchEvent::Stay) {
            spin.observe_switch(decision.inferred_spin_sign, config.spin_confirm_switches);
        }
        update_count++;
    }

    CampColor color { CampColor::UNKNOWN };
    EKF ekf { EKF { } };
    OutpostArmorLayout layout { };
    TimePoint time_stamp;

    bool initialized { false };
    int current_armor_id { kUnknownArmorId };
    SpinTracker spin { };
    int update_count { 0 };
    TrackingConfig config { };
};

OutpostRobotState::OutpostRobotState() noexcept
    : OutpostRobotState(Clock::now()) { }

OutpostRobotState::OutpostRobotState(TimePoint stamp) noexcept
    : pimpl { std::make_unique<Impl>(stamp) } { }

OutpostRobotState::~OutpostRobotState() noexcept = default;

auto OutpostRobotState::initialize(Armor3D const& armor, TimePoint t) -> void {
    return pimpl->initialize(armor, t);
}

auto OutpostRobotState::predict(TimePoint t) -> void { return pimpl->predict(t); }

auto OutpostRobotState::update(std::span<Armor3D const> armors) -> bool {
    return pimpl->update(armors);
}

auto OutpostRobotState::is_converged() const -> bool { return pimpl->is_converged(); }

auto OutpostRobotState::get_snapshot() const -> Snapshot { return pimpl->get_snapshot(); }

auto OutpostRobotState::distance() const -> double { return pimpl->distance(); }
