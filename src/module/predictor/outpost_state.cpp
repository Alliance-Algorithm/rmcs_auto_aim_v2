#include "outpost_state.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>

#include "utility/math/angle.hpp"
#include "utility/math/mahalanobis.hpp"

using namespace rmcs::predictor;

namespace {
constexpr int kUnknownOutpostOrderIdx = -1;
constexpr int kUnknownOutpostSlotIdx  = -1;

struct OutpostObservation {
    OutpostState::EKF::ZVec z;
    OutpostState::EKF::RMat R;
    Eigen::Vector3d xyz;
    Eigen::Vector3d ypr;
    Eigen::Vector3d ypd;
};
}

struct OutpostState::Impl {
    enum class OrderSearchMode {
        AllOrders,
        CommittedOrderOnly,
        NeutralHeightOnly,
    };

    struct SpinState {
        int sign { +1 };
        bool locked { false };
        int candidate_streak { 0 };

        static auto normalize(int spin_sign) -> int { return spin_sign >= 0 ? +1 : -1; }

        auto reset() -> void {
            sign             = +1;
            locked           = false;
            candidate_streak = 0;
        }

        auto current_sign() const -> int { return sign; }

        auto locked_sign() const -> std::optional<int> {
            if (!locked) return std::nullopt;
            return sign;
        }

        auto accept_frame_sign(int frame_sign, int confirm_frames) -> void {
            auto normalized_sign = normalize(frame_sign);
            if (locked) {
                sign = normalized_sign;
                return;
            }

            if (normalized_sign == sign) candidate_streak++;
            else {
                sign             = normalized_sign;
                candidate_streak = 1;
            }

            if (candidate_streak >= confirm_frames) {
                locked = true;
            }
        }
    };

    struct OrderState {
        int committed_idx { kUnknownOutpostOrderIdx };
        int last_candidate_idx { kUnknownOutpostOrderIdx };
        int candidate_streak { 0 };

        auto reset() -> void {
            committed_idx      = kUnknownOutpostOrderIdx;
            last_candidate_idx = kUnknownOutpostOrderIdx;
            candidate_streak   = 0;
        }

        auto has_committed() const -> bool { return committed_idx != kUnknownOutpostOrderIdx; }

        auto current_or_default(int default_order_idx) const -> int {
            if (has_committed()) return committed_idx;
            return default_order_idx;
        }

        auto accept_candidate(int candidate_order_idx, int confirm_frames) -> void {
            if (candidate_order_idx == committed_idx) {
                last_candidate_idx = candidate_order_idx;
                candidate_streak   = 0;
                return;
            }

            auto repeated_candidate = candidate_order_idx == last_candidate_idx;
            last_candidate_idx      = candidate_order_idx;
            candidate_streak =
                repeated_candidate ? std::min(candidate_streak + 1, confirm_frames) : 1;

            if (candidate_streak >= confirm_frames) {
                committed_idx    = candidate_order_idx;
                candidate_streak = 0;
            }
        }
    };

    struct MatchSearchOptions {
        bool single_observation { false };
        OrderSearchMode order_mode { OrderSearchMode::AllOrders };
        std::optional<int> locked_spin_sign;
        int continuity_spin_sign { +1 };
    };

    using Observations   = std::vector<OutpostObservation>;
    using SlotAssignment = std::vector<int>;
    using SlotUsage      = std::array<bool, OutpostEKFParameters::kOutpostArmorCount>;

    struct ObservationErrors {
        double yaw { 0.0 };
        double azimuth { 0.0 };
        double z { 0.0 };
    };

    struct MatchHypothesis {
        int order_idx { 0 };
        int spin_sign { +1 };
    };

    auto reset() -> void {
        order_state.reset();
        spin_state.reset();
        last_visible_slot_idx = kUnknownOutpostSlotIdx;
    }

    auto current_order_idx() const -> int {
        return order_state.current_or_default(neutral_outpost_order_for_slot(0));
    }

    auto current_spin_sign() const -> int { return spin_state.current_sign(); }

    auto spin_locked() const -> bool { return spin_state.locked; }

    static auto normalize_spin_sign(int spin_sign) -> int {
        return SpinState::normalize(spin_sign);
    }

    auto locked_spin_sign() const -> std::optional<int> { return spin_state.locked_sign(); }

    static auto limit_outpost_armors(std::span<Armor3D const> armors) -> std::span<Armor3D const> {
        return armors.first(
            std::min<std::size_t>(armors.size(), OutpostEKFParameters::kOutpostArmorCount));
    }

    static auto build_observations(std::span<Armor3D const> armors) -> Observations {
        auto limited_armors = limit_outpost_armors(armors);
        auto observations   = Observations {};
        observations.reserve(limited_armors.size());
        for (auto const& armor : limited_armors)
            observations.emplace_back(outpost_observation(armor));
        return observations;
    }

    auto resolve_order_search_mode(bool single_observation) const -> OrderSearchMode {
        if (!single_observation) return OrderSearchMode::AllOrders;
        return has_committed_order() ? OrderSearchMode::CommittedOrderOnly
                                     : OrderSearchMode::NeutralHeightOnly;
    }

    auto build_match_search_options(std::size_t observation_count) const -> MatchSearchOptions {
        auto const single_observation = observation_count == 1;
        auto const locked_spin_sign   = this->locked_spin_sign();
        auto const continuity_spin_sign =
            normalize_spin_sign(locked_spin_sign.value_or(current_spin_sign()));
        return {
            single_observation,
            resolve_order_search_mode(single_observation),
            locked_spin_sign,
            continuity_spin_sign,
        };
    }

    auto is_slot_assignment_feasible(
        int slot, int order_idx, MatchSearchOptions const& options) const -> bool {
        if (options.single_observation && last_visible_slot_idx != kUnknownOutpostSlotIdx) {
            auto reachable_slot =
                advance_outpost_slot(last_visible_slot_idx, options.continuity_spin_sign);
            if (slot != last_visible_slot_idx && slot != reachable_slot) return false;
        }

        if (options.order_mode == OrderSearchMode::NeutralHeightOnly
            && OutpostEKFParameters::outpost_height_rank(order_idx, slot) != 0)
            return false;

        return true;
    }

    static auto estimate_center_z(Observations const& observations,
        SlotAssignment const& assignment, MatchHypothesis const& hypothesis) -> double {
        double z_sum = 0.0;
        for (std::size_t i = 0; i < observations.size(); ++i) {
            auto slot = assignment[i];
            z_sum += observations[i].xyz[2]
                - OutpostEKFParameters::outpost_height_rank(hypothesis.order_idx, slot)
                    * kOutpostArmorHeightStep;
        }
        return z_sum / static_cast<double>(observations.size());
    }

    static auto build_assignment_state(Observations const& observations,
        SlotAssignment const& assignment, OutpostState::MatchContext const& context,
        MatchSearchOptions const& options, MatchHypothesis const& hypothesis) -> EKF::XVec {
        auto x_test = hypothesis.spin_sign >= 0 ? context.positive_spin_x : context.negative_spin_x;
        if (!options.single_observation)
            x_test[4] = estimate_center_z(observations, assignment, hypothesis);
        return x_test;
    }

    static auto observation_errors(OutpostObservation const& observation, EKF::XVec const& x_test,
        int slot, int order_idx) -> ObservationErrors {
        auto xyz_pred      = OutpostEKFParameters::h_armor_xyz(x_test, slot, order_idx);
        auto ypd_pred      = util::xyz2ypd(xyz_pred);
        auto yaw_pred      = OutpostEKFParameters::armor_yaw(x_test, slot);
        auto yaw_error     = std::abs(util::normalize_angle(observation.ypr[0] - yaw_pred));
        auto azimuth_error = std::abs(util::normalize_angle(observation.ypd[0] - ypd_pred[0]));
        auto z_error       = std::abs(observation.xyz[2] - xyz_pred[2]);
        return { yaw_error, azimuth_error, z_error };
    }

    auto observation_z_gate(MatchSearchOptions const& options) const -> double {
        return options.single_observation ? outpost_single_observation_z_gate : outpost_z_gate;
    }

    auto passes_observation_gate(
        ObservationErrors const& errors, MatchSearchOptions const& options) const -> bool {
        return errors.yaw <= outpost_yaw_gate && errors.azimuth <= outpost_azimuth_gate
            && errors.z <= observation_z_gate(options);
    }

    auto single_observation_cost(ObservationErrors const& errors) const -> double {
        auto yaw_cost     = errors.yaw / outpost_yaw_gate;
        auto azimuth_cost = errors.azimuth / outpost_azimuth_gate;
        auto z_cost       = errors.z / outpost_single_observation_z_gate;
        return yaw_cost * yaw_cost + azimuth_cost * azimuth_cost
            + outpost_single_observation_z_weight * z_cost * z_cost;
    }

    static auto mahalanobis_cost(OutpostObservation const& observation,
        OutpostState::MatchContext const& context, EKF::XVec const& x_test, int slot, int order_idx)
        -> std::optional<double> {
        auto H          = OutpostEKFParameters::H(x_test, slot, order_idx);
        auto z_hat      = OutpostEKFParameters::h(x_test, slot, order_idx);
        auto innovation = OutpostEKFParameters::z_subtract(observation.z, z_hat);
        auto S          = H * context.P * H.transpose() + observation.R;
        return util::mahalanobis_distance(innovation, S);
    }

    auto observation_cost(OutpostObservation const& observation,
        OutpostState::MatchContext const& context, EKF::XVec const& x_test,
        MatchSearchOptions const& options, int slot, int order_idx) const -> std::optional<double> {
        auto errors = observation_errors(observation, x_test, slot, order_idx);
        if (!passes_observation_gate(errors, options)) return std::nullopt;
        if (options.single_observation) return single_observation_cost(errors);
        return mahalanobis_cost(observation, context, x_test, slot, order_idx);
    }

    auto assignment_cost(Observations const& observations, SlotAssignment const& assignment,
        OutpostState::MatchContext const& context, MatchSearchOptions const& options,
        MatchHypothesis const& hypothesis) const -> std::optional<double> {
        auto x_test =
            build_assignment_state(observations, assignment, context, options, hypothesis);

        double total_cost = 0.0;
        for (std::size_t i = 0; i < observations.size(); ++i) {
            auto cost = observation_cost(
                observations[i], context, x_test, options, assignment[i], hypothesis.order_idx);
            if (!cost.has_value()) return std::nullopt;
            total_cost += cost.value();
        }
        return total_cost;
    }

    auto update_best_match(Observations const& observations, SlotAssignment const& assignment,
        OutpostState::MatchContext const& context, MatchSearchOptions const& options,
        MatchHypothesis const& hypothesis, OutpostState::FrameMatch& best) const -> void {
        auto total_cost = assignment_cost(observations, assignment, context, options, hypothesis);
        if (!total_cost.has_value() || total_cost.value() >= best.cost) return;

        best.valid                = true;
        best.order_idx            = hypothesis.order_idx;
        best.spin_sign            = hypothesis.spin_sign;
        best.cost                 = total_cost.value();
        best.visible_slot_idx     = select_visible_slot(observations, assignment);
        best.slots_by_observation = assignment;
    }

    auto search_slot_assignments(std::size_t obs_index, Observations const& observations,
        OutpostState::MatchContext const& context, MatchSearchOptions const& options,
        MatchHypothesis const& hypothesis, SlotAssignment& assignment, SlotUsage& slot_used,
        OutpostState::FrameMatch& best) const -> void {
        if (obs_index == observations.size()) {
            update_best_match(observations, assignment, context, options, hypothesis, best);
            return;
        }

        auto const slot_count = OutpostEKFParameters::kOutpostArmorCount;
        for (int slot = 0; slot < slot_count; ++slot) {
            if (slot_used[slot]) continue;
            if (!is_slot_assignment_feasible(slot, hypothesis.order_idx, options)) continue;
            slot_used[slot]       = true;
            assignment[obs_index] = slot;
            search_slot_assignments(obs_index + 1, observations, context, options, hypothesis,
                assignment, slot_used, best);
            slot_used[slot]       = false;
            assignment[obs_index] = -1;
        }
    }

    auto search_spin_hypotheses(Observations const& observations,
        OutpostState::MatchContext const& context, MatchSearchOptions const& options, int order_idx,
        SlotAssignment& assignment, SlotUsage& slot_used, OutpostState::FrameMatch& best) const
        -> void {
        if (options.locked_spin_sign.has_value()) {
            auto hypothesis = MatchHypothesis {
                order_idx,
                normalize_spin_sign(options.locked_spin_sign.value()),
            };
            search_slot_assignments(
                0, observations, context, options, hypothesis, assignment, slot_used, best);
            return;
        }

        for (int spin_sign : { -1, +1 }) {
            auto hypothesis = MatchHypothesis { order_idx, spin_sign };
            search_slot_assignments(
                0, observations, context, options, hypothesis, assignment, slot_used, best);
        }
    }

    auto search_order_hypotheses(Observations const& observations,
        OutpostState::MatchContext const& context, MatchSearchOptions const& options,
        SlotAssignment& assignment, SlotUsage& slot_used, OutpostState::FrameMatch& best) const
        -> void {
        if (options.order_mode == OrderSearchMode::CommittedOrderOnly) {
            search_spin_hypotheses(
                observations, context, options, current_order_idx(), assignment, slot_used, best);
            return;
        }

        for (int order_idx = 0; order_idx < OutpostEKFParameters::kOutpostHeightOrderCount;
            ++order_idx) {
            search_spin_hypotheses(
                observations, context, options, order_idx, assignment, slot_used, best);
        }
    }

    auto max_match_cost(MatchSearchOptions const& options) const -> double {
        return options.single_observation ? outpost_single_observation_cost_gate
                                          : outpost_mahalanobis_gate;
    }

    auto match_armor(Armor3D const& armor, OutpostState::MatchContext const& context) const
        -> OutpostState::MatchResult {
        auto frame_match = match_frame(std::span<Armor3D const> { &armor, 1 }, context);
        if (!frame_match.valid) return { -1, 1e10, false };
        return { frame_match.slots_by_observation.front(), frame_match.cost, true };
    }

    auto match_frame(std::span<Armor3D const> armors,
        OutpostState::MatchContext const& context) const -> OutpostState::FrameMatch {
        if (armors.empty()) return {};

        auto observations = build_observations(armors);
        auto best         = OutpostState::FrameMatch {};
        best.cost         = std::numeric_limits<double>::infinity();

        auto const options = build_match_search_options(observations.size());
        auto assignment    = SlotAssignment(observations.size(), -1);
        auto slot_used     = SlotUsage {};
        search_order_hypotheses(observations, context, options, assignment, slot_used, best);

        if (!best.valid || best.cost > max_match_cost(options)) return {};
        return best;
    }

    auto apply_frame_match(OutpostState::FrameMatch const& frame_match) -> void {
        if (!frame_match.valid) return;

        spin_state.accept_frame_sign(frame_match.spin_sign, outpost_spin_confirm_frames);
        last_visible_slot_idx = frame_match.visible_slot_idx;
        if (frame_match.slots_by_observation.size() >= 2)
            order_state.accept_candidate(frame_match.order_idx, outpost_order_confirm_frames);
    }

    auto has_committed_order() const -> bool { return order_state.has_committed(); }

    static auto neutral_outpost_order_for_slot(int slot_idx) -> int {
        for (int order_idx = 0; order_idx < OutpostEKFParameters::kOutpostHeightOrderCount;
            ++order_idx) {
            if (OutpostEKFParameters::outpost_height_rank(order_idx, slot_idx) == 0)
                return order_idx;
        }
        return 0;
    }

    static auto advance_outpost_slot(int slot_idx, int spin_sign) -> int {
        auto normalized_slot = (slot_idx % OutpostEKFParameters::kOutpostArmorCount
                                   + OutpostEKFParameters::kOutpostArmorCount)
            % OutpostEKFParameters::kOutpostArmorCount;
        auto step = spin_sign >= 0 ? -1 : +1;
        return (normalized_slot + step + OutpostEKFParameters::kOutpostArmorCount)
            % OutpostEKFParameters::kOutpostArmorCount;
    }

    static auto select_visible_slot(
        Observations const& observations, SlotAssignment const& assignment) -> int {
        auto best_slot        = kUnknownOutpostSlotIdx;
        auto best_abs_azimuth = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < observations.size(); ++i) {
            if (assignment[i] < 0) continue;
            auto abs_azimuth = std::abs(observations[i].ypd[0]);
            if (abs_azimuth >= best_abs_azimuth) continue;
            best_abs_azimuth = abs_azimuth;
            best_slot        = assignment[i];
        }
        return best_slot;
    }

    static auto outpost_observation(Armor3D const& armor) -> OutpostObservation {
        auto const [pos_x, pos_y, pos_z] = armor.translation;
        auto const xyz                   = Eigen::Vector3d { pos_x, pos_y, pos_z };

        auto const [quat_x, quat_y, quat_z, quat_w] = armor.orientation;
        auto const orientation = Eigen::Quaterniond { quat_w, quat_x, quat_y, quat_z };

        auto const ypr = util::eulers(orientation);
        auto const ypd = util::xyz2ypd(xyz);

        auto z = EKF::ZVec {};
        z << ypd[0], ypd[1], ypd[2], ypr[0];

        return { z, OutpostEKFParameters::R(xyz, ypr, ypd), xyz, ypr, ypd };
    }

    const double outpost_yaw_gate { util::deg2rad(35.0) };
    const double outpost_azimuth_gate { util::deg2rad(35.0) };
    const double outpost_z_gate { 0.06 };
    const double outpost_single_observation_z_gate { 0.18 };
    const double outpost_single_observation_z_weight { 0.15 };
    const double outpost_single_observation_cost_gate { 3.0 };
    const double outpost_mahalanobis_gate { 20.0 };
    const int outpost_spin_confirm_frames { 3 };
    const int outpost_order_confirm_frames { 2 };

    OrderState order_state {};
    SpinState spin_state {};
    int last_visible_slot_idx { kUnknownOutpostSlotIdx };
};

OutpostState::OutpostState() noexcept
    : pimpl { std::make_unique<Impl>() } { }
OutpostState::~OutpostState() noexcept = default;

auto OutpostState::reset() -> void { return pimpl->reset(); }

auto OutpostState::current_order_idx() const -> int { return pimpl->current_order_idx(); }

auto OutpostState::current_spin_sign() const -> int { return pimpl->current_spin_sign(); }

auto OutpostState::spin_locked() const -> bool { return pimpl->spin_locked(); }

auto OutpostState::match_armor(Armor3D const& armor, MatchContext const& context) const
    -> MatchResult {
    return pimpl->match_armor(armor, context);
}

auto OutpostState::match_frame(std::span<Armor3D const> armors, MatchContext const& context) const
    -> FrameMatch {
    return pimpl->match_frame(armors, context);
}

auto OutpostState::apply_frame_match(FrameMatch const& frame_match) -> void {
    return pimpl->apply_frame_match(frame_match);
}
