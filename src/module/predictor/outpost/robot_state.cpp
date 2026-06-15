#include "robot_state.hpp"

#include "module/predictor/outpost/snapshot.hpp"
#include "utility/math/angle.hpp"
#include "utility/time.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

namespace rmcs::predictor {

struct OutpostRobotState::Impl {
    enum class MotionMode { STATIC, CW, CCW };

    struct Candidate {
        int slot_id { 0 };
        double phase_offset { 0.0 };
        double height_offset { 0.0 };
        bool assigned { false };
        std::optional<MotionMode> motion_mode;
    };

    struct CandidateScore {
        Candidate candidate;
        double score { std::numeric_limits<double>::infinity() };
    };

    struct MatchResult {
        OutpostEKFParameters::ArmorObservation observation;
        Candidate candidate;
        double reference_yaw { 0.0 };
        double score { std::numeric_limits<double>::infinity() };
    };

    explicit Impl(TimePoint stamp) noexcept
        : time_stamp { stamp } { }

    auto initialize(Armor3d const& armor, TimePoint t) -> void {
        color        = armor_color2camp_color(armor.color);
        time_stamp   = t;
        update_count = 0;
        motion_mode  = std::nullopt;

        layout = OutpostArmorLayout {};
        assign_slot(0, 0.0, 0.0);
        last_matched_slot_id = 0;

        ekf         = EKF { OutpostEKFParameters::x(armor),
            OutpostEKFParameters::P_initial_dig().asDiagonal() };
        initialized = true;

        auto const observation = OutpostEKFParameters::observe(armor);
        mode_reference_yaw     = observation.ypr[0];
        mode_reference_stamp   = t;
    }

    auto predict(TimePoint t) -> void {
        if (t <= time_stamp) return;

        if (initialized) {
            auto const dt   = util::delta_time(t, time_stamp);
            auto const dt_s = dt.count();
            ekf.predict(
                OutpostEKFParameters::f(dt_s, angular_velocity()),
                [dt_s](EKF::XVec const&) { return OutpostEKFParameters::F(dt_s); },
                OutpostEKFParameters::Q(dt_s));
        }

        time_stamp = t;
    }

    auto update(std::span<Armor3d const> armors) -> bool {
        if (armors.empty()) return false;

        if (!initialized) {
            initialize(armors.front(), time_stamp);
            ++update_count;
            return true;
        }

        auto match = select_best_match(armors);
        if (!match.has_value()) return false;

        if (apply_match(*match)) ++update_count;
        return true;
    }

    auto is_converged() const -> bool {
        if (!initialized) return false;
        if (!std::isfinite(distance())) return false;

        constexpr int min_updates = 3;
        return layout.slots[0].assigned && update_count >= min_updates;
    }

    auto get_snapshot() const -> std::optional<Snapshot> {
        if (!initialized) return std::nullopt;
        return Snapshot { OutpostSnapshot {
            ekf.x, color, time_stamp, layout, angular_velocity() } };
    }

    auto distance() const -> double {
        if (!initialized) return std::numeric_limits<double>::infinity();
        return std::hypot(ekf.x[0], ekf.x[2]);
    }

private:
    static constexpr auto kPhaseOffsets = std::array {
        0.0,
        OutpostEKFParameters::kPhaseStep,
        -OutpostEKFParameters::kPhaseStep,
    };

    static constexpr auto kModeConfirmWindow             = std::chrono::duration<double> { 0.2 };
    static constexpr auto kStaticReferenceYawThreshold   = 0.08;
    static constexpr auto kRotationReferenceYawThreshold = 0.25;

    auto assign_slot(int slot_id, double phase_offset, double height_offset) -> void {
        auto& slot         = layout.slots.at(slot_id);
        slot.phase_offset  = phase_offset;
        slot.height_offset = height_offset;
        slot.assigned      = true;
    }

    static constexpr auto mode_angular_velocity(MotionMode mode) -> double {
        switch (mode) {
        case MotionMode::STATIC:
            return 0.0;
        case MotionMode::CW:
            return -kOutpostAngularSpeed;
        case MotionMode::CCW:
            return kOutpostAngularSpeed;
        }

        return 0.0;
    }

    auto angular_velocity() const -> double {
        if (!motion_mode.has_value()) return 0.0;
        return mode_angular_velocity(*motion_mode);
    }

    static constexpr auto next_slot_id(int slot_id, MotionMode mode) -> int {
        constexpr int slot_count = OutpostEKFParameters::kOutpostArmorCount;
        switch (mode) {
        case MotionMode::CW:
            return (slot_id + 1) % slot_count;
        case MotionMode::CCW:
            return (slot_id + slot_count - 1) % slot_count;
        case MotionMode::STATIC:
            return slot_id;
        }

        return slot_id;
    }

    static constexpr auto height_steps(MotionMode mode) {
        switch (mode) {
        case MotionMode::CW:
            return std::array { -1.0, 2.0 };
        case MotionMode::CCW:
            return std::array { 1.0, -2.0 };
        case MotionMode::STATIC:
            return std::array { 0.0, 0.0 };
        }

        return std::array { 0.0, 0.0 };
    }

    auto candidate_for_assigned_slot(int slot_id) const -> Candidate {
        auto const& slot = layout.slots.at(slot_id);
        return {
            .slot_id       = slot_id,
            .phase_offset  = slot.phase_offset,
            .height_offset = slot.height_offset,
            .assigned      = true,
            .motion_mode   = motion_mode,
        };
    }

    auto candidate_for_slot(int slot_id, double phase_offset, double height_offset,
        MotionMode mode) const -> Candidate {
        return Candidate {
            .slot_id       = slot_id,
            .phase_offset  = phase_offset,
            .height_offset = height_offset,
            .assigned      = layout.slots.at(slot_id).assigned,
            .motion_mode   = mode,
        };
    }

    auto make_candidates() const -> std::vector<Candidate> {
        auto candidates = std::vector<Candidate> {};
        candidates.reserve(OutpostEKFParameters::kOutpostArmorCount * 3);

        for (int slot_id = 0; slot_id < OutpostEKFParameters::kOutpostArmorCount; ++slot_id) {
            if (layout.slots.at(slot_id).assigned) {
                candidates.emplace_back(candidate_for_assigned_slot(slot_id));
            }
        }

        if (motion_mode.has_value()) {
            if (*motion_mode == MotionMode::STATIC) return candidates;
            append_next_slot_candidates(candidates, *motion_mode);
            return candidates;
        }

        append_next_slot_candidates(candidates, MotionMode::CW);
        append_next_slot_candidates(candidates, MotionMode::CCW);
        return candidates;
    }

    auto append_next_slot_candidates(std::vector<Candidate>& candidates, MotionMode mode) const
        -> void {
        auto const source_slot_id = last_matched_slot_id;
        auto const target_slot_id = next_slot_id(source_slot_id, mode);
        auto const& source_slot   = layout.slots.at(source_slot_id);
        auto const& target_slot   = layout.slots.at(target_slot_id);

        if (target_slot.assigned) return;

        for (auto const height_step : height_steps(mode)) {
            candidates.emplace_back(
                candidate_for_slot(target_slot_id, kPhaseOffsets.at(target_slot_id),
                    source_slot.height_offset + height_step * kOutpostArmorHeightStep, mode));
        }
    }

    auto score_candidate(OutpostEKFParameters::ArmorObservation const& observation,
        Candidate const& candidate) const -> double {
        auto const predicted_xyz = OutpostEKFParameters::h_armor_xyz(
            ekf.x, candidate.phase_offset, candidate.height_offset);
        auto const predicted_yaw = OutpostEKFParameters::armor_yaw(ekf.x, candidate.phase_offset);

        auto const position_error = (observation.xyz - predicted_xyz).norm();
        auto const yaw_error = std::abs(util::normalize_angle(observation.ypr[0] - predicted_yaw));
        return position_error + kOutpostRadius * yaw_error;
    }

    auto best_candidate(OutpostEKFParameters::ArmorObservation const& observation) const
        -> std::optional<CandidateScore> {
        auto const candidates = make_candidates();
        if (candidates.empty()) return std::nullopt;

        auto best = CandidateScore {};
        for (auto const& candidate : candidates) {
            auto const score = score_candidate(observation, candidate);
            if (score >= best.score) continue;

            best = {
                .candidate = candidate,
                .score     = score,
            };
        }

        return best;
    }

    auto select_best_match(std::span<Armor3d const> armors) const -> std::optional<MatchResult> {
        auto best_match = std::optional<MatchResult> {};

        for (std::size_t observation_index = 0; observation_index < armors.size();
            ++observation_index) {
            auto const& armor = armors[observation_index];
            if (armor.genre != DeviceId::OUTPOST) continue;

            auto const observation     = OutpostEKFParameters::observe(armor);
            auto const candidate_score = best_candidate(observation);
            if (!candidate_score.has_value()) continue;
            if (best_match.has_value() && candidate_score->score >= best_match->score) continue;

            auto const reference_yaw =
                util::normalize_angle(observation.ypr[0] - candidate_score->candidate.phase_offset);
            best_match = MatchResult {
                .observation   = observation,
                .candidate     = candidate_score->candidate,
                .reference_yaw = reference_yaw,
                .score         = candidate_score->score,
            };
        }

        return best_match;
    }

    auto apply_match(MatchResult const& match) -> bool {
        if (!motion_mode.has_value() && !match.candidate.assigned
            && match.candidate.motion_mode.has_value()) {
            motion_mode = *match.candidate.motion_mode;
            return false;
        }

        update_motion_mode(match);

        auto const& candidate = match.candidate;
        ekf.update(
            OutpostEKFParameters::z(match.observation),
            [candidate](EKF::XVec const& x) {
                return OutpostEKFParameters::h(x, candidate.phase_offset, candidate.height_offset);
            },
            [candidate](EKF::XVec const& x) {
                return OutpostEKFParameters::H(x, candidate.phase_offset, candidate.height_offset);
            },
            OutpostEKFParameters::R(match.observation), OutpostEKFParameters::x_add,
            OutpostEKFParameters::z_subtract);

        if (!layout.slots.at(candidate.slot_id).assigned) {
            assign_slot(candidate.slot_id, candidate.phase_offset, candidate.height_offset);
        }

        last_matched_slot_id = candidate.slot_id;
        return true;
    }

    auto update_motion_mode(MatchResult const& match) -> void {
        if (motion_mode.has_value()) return;

        if (match.candidate.motion_mode.has_value()) {
            motion_mode = *match.candidate.motion_mode;
            return;
        }

        if (util::delta_time(time_stamp, mode_reference_stamp) < kModeConfirmWindow) return;

        auto const mode_delta = util::normalize_angle(match.reference_yaw - mode_reference_yaw);
        auto const abs_delta  = std::abs(mode_delta);
        if (abs_delta < kStaticReferenceYawThreshold) motion_mode = MotionMode::STATIC;
        else if (abs_delta > kRotationReferenceYawThreshold) {
            motion_mode = mode_delta > 0.0 ? MotionMode::CCW : MotionMode::CW;
        }

        mode_reference_yaw   = match.reference_yaw;
        mode_reference_stamp = time_stamp;
    }

    CampColor color { CampColor::UNKNOWN };
    OutpostArmorLayout layout {};
    EKF ekf { EKF {} };
    TimePoint time_stamp;

    bool initialized { false };
    int update_count { 0 };
    int last_matched_slot_id { 0 };

    std::optional<MotionMode> motion_mode;
    double mode_reference_yaw { 0.0 };
    TimePoint mode_reference_stamp;
};

OutpostRobotState::OutpostRobotState() noexcept
    : OutpostRobotState(Clock::now()) { }

OutpostRobotState::OutpostRobotState(TimePoint stamp) noexcept
    : pimpl { std::make_unique<Impl>(stamp) } { }

OutpostRobotState::~OutpostRobotState() noexcept = default;

auto OutpostRobotState::initialize(Armor3d const& armor, TimePoint t) -> void {
    return pimpl->initialize(armor, t);
}

auto OutpostRobotState::predict(TimePoint t) -> void { return pimpl->predict(t); }

auto OutpostRobotState::update(std::span<Armor3d const> armors) -> bool {
    return pimpl->update(armors);
}

auto OutpostRobotState::is_converged() const -> bool { return pimpl->is_converged(); }

auto OutpostRobotState::get_snapshot() const -> std::optional<Snapshot> {
    return pimpl->get_snapshot();
}

auto OutpostRobotState::distance() const -> double { return pimpl->distance(); }

} // namespace rmcs::predictor
