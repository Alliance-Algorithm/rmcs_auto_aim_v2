#include "outpost_robot_state.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <limits>
#include <optional>
#include <span>
#include <string>

#include "utility/logging/printer.hpp"
#include "utility/math/angle.hpp"
#include "utility/math/mahalanobis.hpp"
#include "utility/time.hpp"

using namespace rmcs::predictor;

namespace {

constexpr int kUnknownArmorId = -1;

auto normalize_spin_sign(int spin_sign) -> int {
    if (spin_sign > 0) return +1;
    if (spin_sign < 0) return -1;
    return 0;
}

struct OutpostObservation {
    OutpostRobotState::EKF::ZVec z;
    OutpostRobotState::EKF::RMat R;
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

    auto z = OutpostRobotState::EKF::ZVec {};
    z << ypd[0], ypd[1], ypd[2], ypr[0];

    return { z, OutpostEKFParameters::R(xyz, ypr, ypd), xyz, ypr, ypd };
}

} // namespace

struct OutpostRobotState::Impl {
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

    explicit Impl(Clock::time_point stamp) noexcept
        : time_stamp { stamp } { }

    static auto event_name(SwitchEvent event) -> std::string_view {
        switch (event) {
        case SwitchEvent::Stay:
            return "stay";
        case SwitchEvent::ClockwiseSwitch:
            return "clockwise_switch";
        case SwitchEvent::CounterClockwiseSwitch:
            return "counter_clockwise_switch";
        case SwitchEvent::Invalid:
            return "invalid";
        }
        return "unknown";
    }

    auto initialize(Armor3D const& armor, Clock::time_point t) -> void {
        reset_runtime_state(t);
        if (armor.genre != DeviceId::OUTPOST) return;

        color      = armor_color2camp_color(armor.color);
        ekf        = EKF { OutpostEKFParameters::x(armor),
            OutpostEKFParameters::P_initial_dig().asDiagonal() };
        time_stamp = t;

        layout                   = OutpostArmorLayout {};
        layout.slots[0].assigned = true;

        current_armor_id = 0;
        initialized      = true;

        log_state_summary("initialize", std::nullopt, nullptr, true);
    }

    auto predict(Clock::time_point t) -> void {
        if (initialized) {
            auto dt = util::delta_time(t, time_stamp);
            if (dt > reset_interval) {
                reset_runtime_state(t);
                return;
            }

            auto const dt_s = dt.count();
            ekf.predict(
                OutpostEKFParameters::f(dt_s, current_spin_sign()),
                [dt_s](EKF::XVec const&) { return OutpostEKFParameters::F(dt_s); },
                OutpostEKFParameters::Q(dt_s));
        }

        time_stamp = t;
    }

    // auto match(Armor3D const& armor) const -> MatchResult {
    //     if (!initialized || armor.genre != DeviceId::OUTPOST) return { -1, 1e10, false };
    //
    //     auto const decision = decide_association(make_observation(armor));
    //     return { decision.armor_id, decision.error, decision.is_valid };
    // }

    auto update(Armor3D const& armor) -> bool {
        if (armor.genre != DeviceId::OUTPOST) return false;
        if (!initialized) {
            initialize(armor, time_stamp);
            return true;
        }

        auto const observation = make_observation(armor);
        auto const decision    = decide_association(observation);
        if (!decision.is_valid) return false;

        apply_association(decision, observation);
        return true;
    }

    auto update(std::span<Armor3D const> armors) -> bool {
        auto best_armor    = static_cast<Armor3D const*>(nullptr);
        auto best_decision = AssociationDecision {};

        for (auto const& armor : armors) {
            if (armor.genre != DeviceId::OUTPOST) continue;
            if (!initialized) {
                initialize(armor, time_stamp);
                return true;
            }

            auto const observation = make_observation(armor);
            auto const decision    = decide_association(observation);
            if (!decision.is_valid || decision.error >= best_decision.error) continue;
            best_armor    = &armor;
            best_decision = decision;
        }

        if (best_armor == nullptr) return false;

        apply_association(best_decision, make_observation(*best_armor));
        return true;
    }

    auto is_converged() const -> bool {
        return initialized && spin_locked
            && assigned_count() == OutpostEKFParameters::kOutpostArmorCount
            && update_count >= min_converged_updates;
    }

    auto get_snapshot() const -> Snapshot {
        return { ekf.x, color, assigned_count(), time_stamp, current_spin_sign(), layout };
    }

    auto distance() const -> double { return std::sqrt(ekf.x[0] * ekf.x[0] + ekf.x[2] * ekf.x[2]); }

private:
    auto reset_runtime_state(Clock::time_point t) -> void {
        color                = CampColor::UNKNOWN;
        ekf                  = EKF {};
        layout               = OutpostArmorLayout {};
        time_stamp           = t;
        initialized          = false;
        current_armor_id     = kUnknownArmorId;
        spin_sign            = 0;
        spin_candidate       = 0;
        spin_candidate_count = 0;
        spin_locked          = false;
        update_count         = 0;
    }

    auto assigned_count() const -> int {
        return static_cast<int>(
            std::ranges::count(layout.slots, true, &OutpostArmorSlot::assigned));
    }

    auto has_current_armor() const -> bool {
        return current_armor_id != kUnknownArmorId && layout.slots[current_armor_id].assigned;
    }

    auto current_spin_sign() const -> int {
        if (spin_locked) return spin_sign;
        return spin_candidate;
    }

    static auto switch_phase_delta(int inferred_spin_sign) -> double {
        return -static_cast<double>(normalize_spin_sign(inferred_spin_sign))
            * OutpostEKFParameters::kPhaseStep;
    }

    static auto switch_height_deltas(int inferred_spin_sign) -> std::array<double, 2> {
        if (normalize_spin_sign(inferred_spin_sign) > 0) {
            return { kOutpostArmorHeightStep, -2.0 * kOutpostArmorHeightStep };
        }
        return { -kOutpostArmorHeightStep, 2.0 * kOutpostArmorHeightStep };
    }

    auto find_matching_slot(double phase_offset, double height_offset, int excluded_armor_id) const
        -> int {
        auto best_slot     = kUnknownArmorId;
        auto best_mismatch = std::numeric_limits<double>::infinity();

        for (int id = 0; id < OutpostEKFParameters::kOutpostArmorCount; ++id) {
            if (!layout.slots[id].assigned || id == excluded_armor_id) continue;

            auto const phase_error =
                std::abs(util::normalize_angle(layout.slots[id].phase_offset - phase_offset));
            auto const height_error = std::abs(layout.slots[id].height_offset - height_offset);
            if (phase_error > slot_phase_tolerance || height_error > slot_height_tolerance)
                continue;

            auto const mismatch = phase_error + height_error / kOutpostArmorHeightStep;
            if (mismatch >= best_mismatch) continue;

            best_mismatch = mismatch;
            best_slot     = id;
        }

        return best_slot;
    }

    auto first_unassigned_slot() const -> int {
        for (int id = 0; id < OutpostEKFParameters::kOutpostArmorCount; ++id) {
            if (!layout.slots[id].assigned) return id;
        }
        return kUnknownArmorId;
    }

    auto evaluate_candidate(OutpostObservation const& observation, int armor_id,
        double phase_offset, double height_offset, SwitchEvent event, int inferred_spin_sign,
        bool extends_layout) const -> AssociationDecision {
        auto const predicted_xyz =
            OutpostEKFParameters::h_armor_xyz(ekf.x, phase_offset, height_offset);
        auto const predicted_ypd = util::xyz2ypd(predicted_xyz);
        auto const predicted_yaw = OutpostEKFParameters::armor_yaw(ekf.x, phase_offset);

        auto const yaw_error = std::abs(util::normalize_angle(observation.ypr[0] - predicted_yaw));
        auto const azimuth_error =
            std::abs(util::normalize_angle(observation.ypd[0] - predicted_ypd[0]));
        auto const z_error = std::abs(observation.xyz[2] - predicted_xyz[2]);

        if (yaw_error > yaw_gate || azimuth_error > azimuth_gate || z_error > z_gate) {
            return {};
        }

        auto const H           = OutpostEKFParameters::H(ekf.x, phase_offset, height_offset);
        auto const z_hat       = OutpostEKFParameters::h(ekf.x, phase_offset, height_offset);
        auto const innovation  = OutpostEKFParameters::z_subtract(observation.z, z_hat);
        auto const S           = H * ekf.P() * H.transpose() + observation.R;
        auto const mahalanobis = util::mahalanobis_distance(innovation, S);
        if (!mahalanobis.has_value() || *mahalanobis > mahalanobis_gate) {
            return {};
        }

        auto error = *mahalanobis;
        if (extends_layout) error += layout_extension_penalty;
        if (event == SwitchEvent::Stay) error -= continuity_bonus;

        return { armor_id, error, true, event, normalize_spin_sign(inferred_spin_sign),
            util::normalize_angle(phase_offset), height_offset, extends_layout };
    }

    static auto consider_candidate(
        AssociationDecision const& candidate, AssociationDecision& best_decision) -> void {
        if (!candidate.is_valid || candidate.error >= best_decision.error) return;
        best_decision = candidate;
    }

    auto decide_association(OutpostObservation const& observation) const -> AssociationDecision {
        if (!initialized || !has_current_armor()) return {};

        auto best_decision = AssociationDecision {};

        auto const current_phase  = layout.slots[current_armor_id].phase_offset;
        auto const current_height = layout.slots[current_armor_id].height_offset;
        consider_candidate(evaluate_candidate(observation, current_armor_id, current_phase,
                               current_height, SwitchEvent::Stay, 0, false),
            best_decision);

        auto try_switch_candidates = [&](int inferred_spin_sign, SwitchEvent event) {
            auto const candidate_phase =
                util::normalize_angle(current_phase + switch_phase_delta(inferred_spin_sign));

            for (auto const height_delta : switch_height_deltas(inferred_spin_sign)) {
                auto const candidate_height = current_height + height_delta;

                auto armor_id =
                    find_matching_slot(candidate_phase, candidate_height, current_armor_id);
                auto extends_layout = false;
                if (armor_id == kUnknownArmorId) {
                    armor_id       = first_unassigned_slot();
                    extends_layout = armor_id != kUnknownArmorId;
                }
                if (armor_id == kUnknownArmorId) continue;

                consider_candidate(evaluate_candidate(observation, armor_id, candidate_phase,
                                       candidate_height, event, inferred_spin_sign, extends_layout),
                    best_decision);
            }
        };

        if (spin_locked) {
            auto const event =
                spin_sign > 0 ? SwitchEvent::CounterClockwiseSwitch : SwitchEvent::ClockwiseSwitch;
            try_switch_candidates(spin_sign, event);
        } else {
            try_switch_candidates(-1, SwitchEvent::ClockwiseSwitch);
            try_switch_candidates(+1, SwitchEvent::CounterClockwiseSwitch);
        }

        return best_decision;
    }

    auto update_spin_estimate(int inferred_spin_sign) -> void {
        auto const normalized = normalize_spin_sign(inferred_spin_sign);
        if (normalized == 0 || spin_locked) return;

        if (spin_candidate == normalized) {
            spin_candidate_count++;
        } else {
            spin_candidate       = normalized;
            spin_candidate_count = 1;
        }

        if (spin_candidate_count < spin_confirm_switches) return;

        spin_sign   = spin_candidate;
        spin_locked = true;
    }

    auto format_layout_summary(OutpostArmorLayout const& layout_to_format) const -> std::string {
        auto summary = std::string {};
        for (int id = 0; id < OutpostEKFParameters::kOutpostArmorCount; ++id) {
            auto const& slot = layout_to_format.slots[id];
            auto const predicted_z =
                slot.assigned ? OutpostEKFParameters::h_armor_z(ekf.x, layout_to_format, id) : 0.0;
            summary += std::format("slot{}{{assigned={},dz={:.3f},z={:.3f},phase={:.3f}}}", id,
                slot.assigned, slot.height_offset, predicted_z, slot.phase_offset);
            if (id + 1 < OutpostEKFParameters::kOutpostArmorCount) summary += " ";
        }
        return summary;
    }

    auto log_state_summary(std::string_view stage,
        std::optional<AssociationDecision> const& decision, OutpostObservation const* observation,
        bool force) -> void {
        auto const next_tick = diagnostic_tick + 1;
        auto should_log      = force || (next_tick % diagnostic_log_stride == 0);

        if (decision.has_value() && observation != nullptr
            && decision->armor_id != kUnknownArmorId) {
            auto const predicted_z =
                OutpostEKFParameters::h_armor_z(ekf.x, layout, decision->armor_id);
            auto const z_residual = observation->xyz[2] - predicted_z;
            should_log            = should_log || decision->extends_layout
                || std::abs(z_residual) > diagnostic_z_residual_alert;

            if (should_log) {
                log.info("[{}] event={} armor_id={} spin={} locked={} x4={:.3f} obs_z={:.3f} "
                         "pred_z={:.3f} residual_z={:.3f} layout={}",
                    stage, event_name(decision->event), decision->armor_id, current_spin_sign(),
                    spin_locked, ekf.x[4], observation->xyz[2], predicted_z, z_residual,
                    format_layout_summary(layout));
            }
        } else if (should_log) {
            log.info("[{}] spin={} locked={} x4={:.3f} layout={}", stage, current_spin_sign(),
                spin_locked, ekf.x[4], format_layout_summary(layout));
        }

        diagnostic_tick = next_tick;
    }

    auto apply_association(
        AssociationDecision const& decision, OutpostObservation const& observation) -> void {
        auto next_layout = layout;
        if (decision.extends_layout) {
            next_layout.slots[decision.armor_id].phase_offset  = decision.phase_offset;
            next_layout.slots[decision.armor_id].height_offset = decision.height_offset;
            next_layout.slots[decision.armor_id].assigned      = true;
        }

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
            update_spin_estimate(decision.inferred_spin_sign);
        }
        update_count++;

        log_state_summary("apply_association", decision, &observation, decision.extends_layout);
    }

    CampColor color { CampColor::UNKNOWN };
    EKF ekf { EKF {} };
    OutpostArmorLayout layout {};
    Clock::time_point time_stamp;

    bool initialized { false };
    int current_armor_id { kUnknownArmorId };

    int spin_sign { 0 };
    int spin_candidate { 0 };
    int spin_candidate_count { 0 };
    bool spin_locked { false };

    int update_count { 0 };
    std::size_t diagnostic_tick { 0 };

    const std::chrono::duration<double> reset_interval { 1.0 };

    // 先用相位与高度做离散事件约束，再用观测残差与马氏距离筛掉假匹配。
    const double yaw_gate { util::deg2rad(40.0) };
    const double azimuth_gate { util::deg2rad(35.0) };
    const double z_gate { 0.05 };
    const double slot_phase_tolerance { util::deg2rad(8.0) };
    const double slot_height_tolerance { 0.02 };
    const double mahalanobis_gate { 25.0 };
    const double layout_extension_penalty { 0.25 };
    const double continuity_bonus { 0.10 };
    const double diagnostic_z_residual_alert { 0.03 };
    const int spin_confirm_switches { 2 };
    const int min_converged_updates { 6 };
    const std::size_t diagnostic_log_stride { 20 };

    Printer log { "OutpostRobotState" };
};

OutpostRobotState::OutpostRobotState() noexcept
    : OutpostRobotState(Clock::now()) { }

OutpostRobotState::OutpostRobotState(Clock::time_point stamp) noexcept
    : pimpl { std::make_unique<Impl>(stamp) } { }

OutpostRobotState::~OutpostRobotState() noexcept                                      = default;
OutpostRobotState::OutpostRobotState(OutpostRobotState&&) noexcept                    = default;
auto OutpostRobotState::operator=(OutpostRobotState&&) noexcept -> OutpostRobotState& = default;

auto OutpostRobotState::initialize(Armor3D const& armor, Clock::time_point t) -> void {
    return pimpl->initialize(armor, t);
}

auto OutpostRobotState::predict(Clock::time_point t) -> void { return pimpl->predict(t); }

auto OutpostRobotState::match(Armor3D const& armor) const -> MatchResult {
    // return pimpl->match(armor);
}

auto OutpostRobotState::update(Armor3D const& armor) -> bool { return pimpl->update(armor); }

auto OutpostRobotState::update(std::span<Armor3D const> armors) -> bool {
    return pimpl->update(armors);
}

auto OutpostRobotState::is_converged() const -> bool { return pimpl->is_converged(); }

auto OutpostRobotState::get_snapshot() const -> Snapshot { return pimpl->get_snapshot(); }

auto OutpostRobotState::distance() const -> double { return pimpl->distance(); }
