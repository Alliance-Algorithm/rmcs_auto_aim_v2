#include "robot_state.hpp"

#include "module/predictor/outpost/snapshot.hpp"
#include "utility/math/angle.hpp"
#include "utility/time.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

namespace rmcs::predictor {

struct OutpostRobotState::Impl {
    using Params = OutpostEKFParameters;

    enum class MotionMode { STATIC, CW, CCW };

    struct Candidate {
        int slot_id { 0 };
        double phase_offset { 0.0 };
        double height_offset { 0.0 };
        bool assigned { false };
        std::optional<MotionMode> motion_mode;
    };

    struct MatchResult {
        Params::ArmorObservation observation;
        Candidate candidate;
        double reference_yaw { 0.0 };
        double score { std::numeric_limits<double>::infinity() };
    };

    struct RotationTopology {
        double angular_velocity { 0.0 };
        int slot_delta { 0 };
        double phase_delta { 0.0 };
        std::array<double, 2> height_steps {};
    };

    struct RotationEvidence {
        std::optional<MotionMode> mode;
        int count { 0 };
    };

    explicit Impl(TimePoint stamp) noexcept
        : time_stamp { stamp } { }

    auto initialize(Armor3d const& armor, TimePoint t) -> void {
        color             = armor_color2camp_color(armor.color);
        time_stamp        = t;
        update_count      = 0;
        motion_mode       = std::nullopt;
        rotation_evidence = {};

        layout = OutpostArmorLayout {};
        assign_slot(0, 0.0, 0.0);
        last_matched_slot_id = 0;

        ekf = EKF { Params::x(Params::observe(armor)), Params::P_initial_dig().asDiagonal() };
        initialized = true;

        auto const observation = Params::observe(armor);
        mode_reference_yaw     = observation.ypr[0];
        mode_reference_stamp   = t;
    }

    auto predict(TimePoint t) -> void {
        if (t <= time_stamp) return;

        if (initialized) {
            auto const dt   = util::delta_time(t, time_stamp);
            auto const dt_s = dt.count();
            ekf.predict(
                Params::f(dt_s, angular_velocity()),
                [dt_s](EKF::XVec const&) { return Params::F(dt_s); }, Params::Q(dt_s));
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
    static constexpr auto kModeConfirmWindow            = std::chrono::duration<double> { 0.2 };
    static constexpr auto kStaticYawDeltaThreshold      = util::deg2rad(5.0);
    static constexpr auto kRotationYawDeltaThreshold    = util::deg2rad(15.0);
    static constexpr int kRotationConfirmCandidateCount = 3;

    auto assign_slot(int slot_id, double phase_offset, double height_offset) -> void {
        auto& slot         = layout.slots.at(slot_id);
        slot.phase_offset  = phase_offset;
        slot.height_offset = height_offset;
        slot.assigned      = true;
    }

    auto reset_state(int slot_id, Params::ArmorObservation const& obs) -> void {
        ekf = EKF { Params::x(obs), Params::P_initial_dig().asDiagonal() };

        layout = OutpostArmorLayout {};
        assign_slot(slot_id, 0.0, 0.0);
        last_matched_slot_id = slot_id;

        motion_mode       = std::nullopt;
        rotation_evidence = {};
        update_count      = 0;

        mode_reference_yaw   = obs.ypr[0];
        mode_reference_stamp = time_stamp;
    }

    auto is_layout_consistent() const -> bool {
        if (!layout.slots[0].assigned || !layout.slots[1].assigned || !layout.slots[2].assigned)
            return true;

        auto height_step = [](double height_offset) {
            return static_cast<int>(std::round(height_offset / kOutpostArmorHeightStep));
        };

        auto s0 = height_step(layout.slots[0].height_offset);
        auto s1 = height_step(layout.slots[1].height_offset);
        auto s2 = height_step(layout.slots[2].height_offset);

        auto min_step = std::min({ s0, s1, s2 });
        auto max_step = std::max({ s0, s1, s2 });

        return max_step - min_step == 2 && s0 != s1 && s1 != s2 && s0 != s2;
    }

    static constexpr auto topology_of(MotionMode mode) -> std::optional<RotationTopology> {
        switch (mode) {
        case MotionMode::CW:
            return RotationTopology {
                .angular_velocity = -kOutpostAngularSpeed,
                .slot_delta       = 1,
                .phase_delta      = Params::kPhaseStep,
                .height_steps     = { -1.0, 2.0 },
            };
        case MotionMode::CCW:
            return RotationTopology {
                .angular_velocity = kOutpostAngularSpeed,
                .slot_delta       = -1,
                .phase_delta      = -Params::kPhaseStep,
                .height_steps     = { 1.0, -2.0 },
            };
        case MotionMode::STATIC:
            return std::nullopt;
        }

        return std::nullopt;
    }

    auto angular_velocity() const -> double {
        if (!motion_mode.has_value()) return 0.0;
        auto const topology = topology_of(*motion_mode);
        if (!topology.has_value()) return 0.0;
        return topology->angular_velocity;
    }

    static constexpr auto normalize_slot_id(int slot_id) -> int {
        constexpr int slot_count = Params::kOutpostArmorCount;
        return (slot_id + slot_count) % slot_count;
    }

    auto make_candidates() const -> std::vector<Candidate> {
        auto candidates = std::vector<Candidate> {};
        candidates.reserve(Params::kOutpostArmorCount * 3);

        for (int slot_id = 0; slot_id < Params::kOutpostArmorCount; ++slot_id) {
            auto const& slot = layout.slots.at(slot_id);
            if (!slot.assigned) continue;

            candidates.emplace_back(Candidate {
                .slot_id       = slot_id,
                .phase_offset  = slot.phase_offset,
                .height_offset = slot.height_offset,
                .assigned      = true,
                .motion_mode   = motion_mode,
            });
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
        auto const topology = topology_of(mode);
        if (!topology.has_value()) return;

        auto const source_slot_id = last_matched_slot_id;
        auto const target_slot_id = normalize_slot_id(source_slot_id + topology->slot_delta);
        auto const& source_slot   = layout.slots.at(source_slot_id);
        auto const& target_slot   = layout.slots.at(target_slot_id);

        if (target_slot.assigned) return;

        for (auto const height_step : topology->height_steps) {
            auto const phase_offset =
                util::normalize_angle(source_slot.phase_offset + topology->phase_delta);
            auto const height_offset =
                source_slot.height_offset + height_step * kOutpostArmorHeightStep;

            candidates.emplace_back(Candidate {
                .slot_id       = target_slot_id,
                .phase_offset  = phase_offset,
                .height_offset = height_offset,
                .assigned      = false,
                .motion_mode   = mode,
            });
        }
    }

    auto select_best_match(std::span<Armor3d const> armors) const -> std::optional<MatchResult> {
        auto const candidates = make_candidates();
        if (candidates.empty()) return std::nullopt;

        auto best_match = std::optional<MatchResult> {};

        for (auto const& armor : armors) {
            auto const observation = Params::observe(armor);
            for (auto const& candidate : candidates) {
                // 预测装甲板位置
                auto const predicted_xyz =
                    Params::h_armor_xyz(ekf.x, candidate.phase_offset, candidate.height_offset);
                auto const predicted_yaw = Params::armor_yaw(ekf.x, candidate.phase_offset);

                //  计算匹配分数
                auto const position_error = (observation.xyz - predicted_xyz).norm();
                auto const yaw_error =
                    std::abs(util::normalize_angle(observation.ypr[0] - predicted_yaw));
                auto const score = position_error + kOutpostRadius * yaw_error;

                if (best_match.has_value() && score >= best_match->score) continue;

                best_match = MatchResult {
                    .observation = observation,
                    .candidate   = candidate,
                    .reference_yaw =
                        util::normalize_angle(observation.ypr[0] - candidate.phase_offset),
                    .score = score,
                };
            }
        }

        return best_match;
    }

    auto apply_match(MatchResult const& match) -> bool {
        auto const& candidate = match.candidate;

        if (!motion_mode.has_value() && !candidate.assigned && candidate.motion_mode.has_value()) {
            update_rotation_evidence(*candidate.motion_mode);
            return false;
        }

        rotation_evidence = {};
        update_motion_mode(match);

        ekf.update(
            Params::z(match.observation),
            [candidate](EKF::XVec const& x) {
                return Params::h(x, candidate.phase_offset, candidate.height_offset);
            },
            [candidate](EKF::XVec const& x) {
                return Params::H(x, candidate.phase_offset, candidate.height_offset);
            },
            Params::R(match.observation), Params::x_add, Params::z_subtract);

        assign_slot(candidate.slot_id, candidate.phase_offset, candidate.height_offset);
        last_matched_slot_id = candidate.slot_id;

        if (!is_layout_consistent()) {
            reset_state(candidate.slot_id, match.observation);
        }

        return true;
    }

    auto update_rotation_evidence(MotionMode candidate_mode) -> void {
        if (rotation_evidence.mode == candidate_mode) {
            ++rotation_evidence.count;
        } else {
            rotation_evidence = {
                .mode  = candidate_mode,
                .count = 1,
            };
        }

        if (rotation_evidence.count >= kRotationConfirmCandidateCount) {
            motion_mode       = candidate_mode;
            rotation_evidence = {};
        }
    }

    auto update_motion_mode(MatchResult const& match) -> void {
        // 使用拓扑候选直接确认方向
        if (!motion_mode.has_value() && match.candidate.motion_mode.has_value()) {
            motion_mode = *match.candidate.motion_mode;
            return;
        }

        //  按时间窗口判断 yaw 变化
        auto const elapsed = util::delta_time(time_stamp, mode_reference_stamp);
        if (elapsed < kModeConfirmWindow) return;

        auto const mode_delta = util::normalize_angle(match.reference_yaw - mode_reference_yaw);
        auto const abs_delta  = std::abs(mode_delta);

        // 根据 yaw 变化推断当前观测到的运动模式
        auto observed_motion_mode = std::optional<MotionMode> {};
        if (abs_delta < kStaticYawDeltaThreshold) observed_motion_mode = MotionMode::STATIC;
        else if (abs_delta > kRotationYawDeltaThreshold) {
            observed_motion_mode = mode_delta > 0.0 ? MotionMode::CCW : MotionMode::CW;
        }

        //  应用观测结果
        if (observed_motion_mode.has_value()) {
            if (!motion_mode.has_value()) {
                motion_mode = *observed_motion_mode;
            } else if (*motion_mode != *observed_motion_mode) {
                motion_mode       = std::nullopt;
                rotation_evidence = {};
            }
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
    RotationEvidence rotation_evidence;
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
