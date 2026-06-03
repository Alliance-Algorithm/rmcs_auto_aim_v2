#include "robot_state.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <span>

#include "module/predictor/outpost/snapshot.hpp"
#include "utility/math/mahalanobis.hpp"
#include "utility/time.hpp"

using namespace rmcs::predictor;

namespace {

constexpr int kUnknownArmorId = -1;
using OutpostEKF              = OutpostRobotState::EKF;

struct OutpostObservation {
    OutpostEKF::ZVec z;
    OutpostEKF::RMat R;
    Eigen::Vector3d xyz;
};

struct MatchCandidate {
    int armor_id { kUnknownArmorId };
    double phase_offset { 0.0 };
    double height_offset { 0.0 };
    double error { std::numeric_limits<double>::infinity() };
    bool extends_layout { false };
    bool is_valid { false };
};

// 模板 A: slot1 = +H, slot2 = -H
// 模板 B: slot1 = -H, slot2 = +H
enum class HeightTemplate { Unknown, PositiveOnSlot1, NegativeOnSlot1 };

auto resolve_template(OutpostArmorLayout const& layout) -> HeightTemplate {
    if (layout.slots[1].assigned) {
        return layout.slots[1].height_offset > 0.0 ? HeightTemplate::PositiveOnSlot1
                                                   : HeightTemplate::NegativeOnSlot1;
    }
    if (layout.slots[2].assigned) {
        return layout.slots[2].height_offset > 0.0 ? HeightTemplate::NegativeOnSlot1
                                                   : HeightTemplate::PositiveOnSlot1;
    }
    return HeightTemplate::Unknown;
}

auto make_observation(rmcs::Armor3D const& armor) -> OutpostObservation {
    auto const [pos_x, pos_y, pos_z] = armor.translation;
    auto const xyz                   = Eigen::Vector3d { pos_x, pos_y, pos_z };

    auto const [quat_x, quat_y, quat_z, quat_w] = armor.orientation;
    auto const orientation = Eigen::Quaterniond { quat_w, quat_x, quat_y, quat_z };

    auto const ypr = rmcs::util::eulers(orientation);
    auto const ypd = rmcs::util::xyz2ypd(xyz);

    auto z = OutpostEKF::ZVec {};
    z << ypd[0], ypd[1], ypd[2], ypr[0];

    return { z, OutpostEKFParameters::R(xyz, ypr, ypd), xyz };
}

auto assigned_count(OutpostArmorLayout const& layout) -> int {
    return static_cast<int>(std::ranges::count(layout.slots, true, &OutpostArmorSlot::assigned));
}

auto layout_after_match(OutpostArmorLayout const& layout, MatchCandidate const& candidate)
    -> OutpostArmorLayout {
    auto next_layout = layout;
    if (!candidate.extends_layout) return next_layout;

    next_layout.slots[candidate.armor_id].phase_offset  = candidate.phase_offset;
    next_layout.slots[candidate.armor_id].height_offset = candidate.height_offset;
    next_layout.slots[candidate.armor_id].assigned      = true;
    return next_layout;
}

auto slot_phase_offset(int armor_id) -> double {
    switch (armor_id) {
    case 0:
        return 0.0;
    case 1:
        return OutpostEKFParameters::kPhaseStep;
    case 2:
        return -OutpostEKFParameters::kPhaseStep;
    default:
        return 0.0;
    }
}

auto generate_slot_candidates(OutpostArmorLayout const& layout) -> std::array<MatchCandidate, 5> {
    using rmcs::kOutpostArmorHeightStep;
    constexpr auto kPhaseStep = OutpostEKFParameters::kPhaseStep;

    auto candidates = std::array<MatchCandidate, 5> {};
    auto n          = int { 0 };

    for (int id = 0; id < OutpostEKFParameters::kOutpostArmorCount; ++id) {
        if (!layout.slots[id].assigned) continue;
        candidates[n++] = MatchCandidate { id, slot_phase_offset(id),
            layout.slots[id].height_offset, 0.0, false, false };
    }

    if (n == OutpostEKFParameters::kOutpostArmorCount) return candidates;

    auto const tmpl = resolve_template(layout);

    for (int id : { 1, 2 }) {
        if (layout.slots[id].assigned) continue;

        auto const phase = (id == 1) ? +kPhaseStep : -kPhaseStep;

        if (tmpl == HeightTemplate::Unknown) {
            candidates[n++] =
                MatchCandidate { id, phase, +kOutpostArmorHeightStep, 0.0, true, false };
            candidates[n++] =
                MatchCandidate { id, phase, -kOutpostArmorHeightStep, 0.0, true, false };
            continue;
        }

        auto const h1   = (tmpl == HeightTemplate::PositiveOnSlot1) ? +kOutpostArmorHeightStep
                                                                    : -kOutpostArmorHeightStep;
        candidates[n++] = MatchCandidate { id, phase, (id == 1) ? h1 : -h1, 0.0, true, false };
    }

    return candidates;
}

auto evaluate_candidate(OutpostObservation const& observation, OutpostEKF::XVec const& x,
    OutpostEKF::PMat const& P, double mahalanobis_gate, MatchCandidate const& candidate)
    -> MatchCandidate {
    auto result = candidate;
    if (result.armor_id == kUnknownArmorId) return result;

    auto const H           = OutpostEKFParameters::H(x, result.phase_offset, result.height_offset);
    auto const z_hat       = OutpostEKFParameters::h(x, result.phase_offset, result.height_offset);
    auto const innovation  = OutpostEKFParameters::z_subtract(observation.z, z_hat);
    auto const S           = H * P * H.transpose() + observation.R;
    auto const mahalanobis = rmcs::util::mahalanobis_distance(innovation, S);
    if (!mahalanobis.has_value() || *mahalanobis > mahalanobis_gate) return result;

    result.error    = *mahalanobis;
    result.is_valid = true;
    return result;
}

} // namespace

struct OutpostRobotState::Impl {
    explicit Impl(TimePoint stamp) noexcept
        : time_stamp { stamp } { }

    auto initialize(Armor3D const& armor, TimePoint t) -> void {
        color      = armor_color2camp_color(armor.color);
        ekf        = EKF { OutpostEKFParameters::x(armor),
            OutpostEKFParameters::P_initial_dig().asDiagonal() };
        time_stamp = t;

        layout                        = OutpostArmorLayout {};
        layout.slots[0].phase_offset  = 0.0;
        layout.slots[0].height_offset = 0.0;
        layout.slots[0].assigned      = true;
        update_count                  = 0;
        initialized                   = true;
    }

    auto predict(TimePoint t) -> void {
        if (t <= time_stamp) return;

        if (initialized) {
            auto dt = rmcs::util::delta_time(t, time_stamp);
            if (dt > reset_interval) {
                reset_runtime_state(t);
                return;
            }

            auto const dt_s = dt.count();
            ekf.predict(
                OutpostEKFParameters::f(dt_s),
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

        apply_match(best_match->first, best_match->second);
        return true;
    }

    auto is_converged() const -> bool {
        if (!initialized || update_count < 5) return false;
        auto const omega   = std::abs(ekf.x[6]);
        auto const p_omega = ekf.P()(6, 6);
        return p_omega < 4.0 && (omega < 0.3 || std::abs(omega - rmcs::kOutpostAngularSpeed) < 0.5);
    }

    auto get_snapshot() const -> Snapshot {
        return detail::make_outpost_snapshot(
            ekf.x, color, assigned_count(layout), time_stamp, layout);
    }

    auto distance() const -> double {
        return initialized ? std::sqrt(ekf.x[0] * ekf.x[0] + ekf.x[2] * ekf.x[2])
                           : std::numeric_limits<double>::infinity();
    }

private:
    auto reset_runtime_state(TimePoint t) -> void {
        color        = CampColor::UNKNOWN;
        ekf          = EKF {};
        layout       = OutpostArmorLayout {};
        time_stamp   = t;
        initialized  = false;
        update_count = 0;
    }

    auto select_best_match(std::span<Armor3D const> armors) const
        -> std::optional<std::pair<OutpostObservation, MatchCandidate>> {
        auto best_match       = std::optional<std::pair<OutpostObservation, MatchCandidate>> {};
        auto const candidates = generate_slot_candidates(layout);

        for (auto const& armor : armors) {
            auto const observation = make_observation(armor);
            for (auto const& candidate : candidates) {
                auto match =
                    evaluate_candidate(observation, ekf.x, ekf.P(), mahalanobis_gate, candidate);
                if (!match.is_valid) continue;
                if (best_match.has_value() && match.error >= best_match->second.error) continue;

                best_match = std::pair { observation, match };
            }
        }

        return best_match;
    }

    auto apply_match(OutpostObservation const& observation, MatchCandidate const& match) -> void {
        auto const next_layout = layout_after_match(layout, match);

        ekf.update(
            observation.z,
            [layout = next_layout, armor_id = match.armor_id](
                EKF::XVec const& x) { return OutpostEKFParameters::h(x, layout, armor_id); },
            [layout = next_layout, armor_id = match.armor_id](
                EKF::XVec const& x) { return OutpostEKFParameters::H(x, layout, armor_id); },
            observation.R, OutpostEKFParameters::x_add, OutpostEKFParameters::z_subtract);

        layout = next_layout;
        update_count++;
    }

    CampColor color { CampColor::UNKNOWN };
    EKF ekf { EKF {} };
    OutpostArmorLayout layout {};
    TimePoint time_stamp;

    bool initialized { false };
    int update_count { 0 };
    std::chrono::duration<double> reset_interval { 1.5 };
    double mahalanobis_gate { 25.0 };
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
