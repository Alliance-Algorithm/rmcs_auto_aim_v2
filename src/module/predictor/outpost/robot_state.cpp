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

struct SlotCandidate {
    int armor_id { kUnknownArmorId };
    double phase_offset { 0.0 };
    double height_offset { 0.0 };
    bool extends_layout { false };
};

struct MatchDecision {
    int armor_id { kUnknownArmorId };
    double phase_offset { 0.0 };
    double height_offset { 0.0 };
    double error { std::numeric_limits<double>::infinity() };
    bool extends_layout { false };
    bool is_valid { false };
};

struct BestMatch {
    OutpostObservation observation;
    MatchDecision decision;
};

struct MatchingConfig {
    double mahalanobis_gate { 25.0 };
};

struct TrackingConfig {
    std::chrono::duration<double> reset_interval { 1.5 };
    MatchingConfig matching {};
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

auto layout_after_match(OutpostArmorLayout const& layout, MatchDecision const& decision)
    -> OutpostArmorLayout {
    auto next_layout = layout;
    if (!decision.extends_layout) return next_layout;

    next_layout.slots[decision.armor_id].phase_offset  = decision.phase_offset;
    next_layout.slots[decision.armor_id].height_offset = decision.height_offset;
    next_layout.slots[decision.armor_id].assigned      = true;
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

auto generate_slot_candidates(OutpostArmorLayout const& layout) -> std::array<SlotCandidate, 5> {
    using rmcs::kOutpostArmorHeightStep;

    auto candidates = std::array<SlotCandidate, 5> {};
    auto append_at  = int { 0 };
    auto append     = [&](int armor_id, double height_offset, bool extends_layout) {
        candidates[append_at++] = {
            armor_id,
            slot_phase_offset(armor_id),
            height_offset,
            extends_layout,
        };
    };

    for (int armor_id = 0; armor_id < OutpostEKFParameters::kOutpostArmorCount; ++armor_id) {
        if (!layout.slots[armor_id].assigned) continue;
        append(armor_id, layout.slots[armor_id].height_offset, false);
    }

    if (assigned_count(layout) == OutpostEKFParameters::kOutpostArmorCount) return candidates;

    auto const height_slot1 = [&](HeightTemplate t) -> double {
        return t == HeightTemplate::PositiveOnSlot1 ? kOutpostArmorHeightStep
                                                    : -kOutpostArmorHeightStep;
    };

    auto const tmpl = resolve_template(layout);

    if (!layout.slots[1].assigned) {
        if (tmpl == HeightTemplate::Unknown) {
            append(1, kOutpostArmorHeightStep, true);
            append(1, -kOutpostArmorHeightStep, true);
        } else {
            append(1, height_slot1(tmpl), true);
        }
    }

    if (!layout.slots[2].assigned) {
        if (tmpl == HeightTemplate::Unknown) {
            append(2, kOutpostArmorHeightStep, true);
            append(2, -kOutpostArmorHeightStep, true);
        } else {
            append(2, -height_slot1(tmpl), true);
        }
    }

    return candidates;
}

auto evaluate_candidate(OutpostObservation const& observation, OutpostEKF::XVec const& x,
    OutpostEKF::PMat const& P, MatchingConfig const& config, SlotCandidate const& candidate)
    -> MatchDecision {
    if (candidate.armor_id == kUnknownArmorId) return {};

    auto const H     = OutpostEKFParameters::H(x, candidate.phase_offset, candidate.height_offset);
    auto const z_hat = OutpostEKFParameters::h(x, candidate.phase_offset, candidate.height_offset);
    auto const innovation  = OutpostEKFParameters::z_subtract(observation.z, z_hat);
    auto const S           = H * P * H.transpose() + observation.R;
    auto const mahalanobis = rmcs::util::mahalanobis_distance(innovation, S);
    if (!mahalanobis.has_value() || *mahalanobis > config.mahalanobis_gate) return {};

    return { candidate.armor_id, candidate.phase_offset, candidate.height_offset, *mahalanobis,
        candidate.extends_layout, true };
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
            if (dt > config.reset_interval) {
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

        apply_match(*best_match);
        return true;
    }

    auto is_converged() const -> bool {
        if (!initialized || update_count < 5) return false;
        auto const omega   = std::abs(ekf.x[6]);
        auto const p_omega = ekf.P()(6, 6);
        // return true;
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

    auto select_best_match(std::span<Armor3D const> armors) const -> std::optional<BestMatch> {
        auto best_match       = std::optional<BestMatch> {};
        auto const candidates = generate_slot_candidates(layout);

        for (auto const& armor : armors) {
            auto const observation = make_observation(armor);
            for (auto const& candidate : candidates) {
                auto const decision =
                    evaluate_candidate(observation, ekf.x, ekf.P(), config.matching, candidate);
                if (!decision.is_valid) continue;
                if (best_match.has_value() && decision.error >= best_match->decision.error)
                    continue;

                best_match = BestMatch { observation, decision };
            }
        }

        return best_match;
    }

    auto apply_match(BestMatch const& best_match) -> void {
        auto const next_layout = layout_after_match(layout, best_match.decision);

        ekf.update(
            best_match.observation.z,
            [layout = next_layout, armor_id = best_match.decision.armor_id](
                EKF::XVec const& x) { return OutpostEKFParameters::h(x, layout, armor_id); },
            [layout = next_layout, armor_id = best_match.decision.armor_id](
                EKF::XVec const& x) { return OutpostEKFParameters::H(x, layout, armor_id); },
            best_match.observation.R, OutpostEKFParameters::x_add,
            OutpostEKFParameters::z_subtract);

        layout = next_layout;
        update_count++;
    }

    CampColor color { CampColor::UNKNOWN };
    EKF ekf { EKF {} };
    OutpostArmorLayout layout {};
    TimePoint time_stamp;

    bool initialized { false };
    int update_count { 0 };
    TrackingConfig config {};
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
