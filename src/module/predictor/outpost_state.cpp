#include "outpost_state.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>

#include "utility/math/angle.hpp"

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
                sign             = normalized_sign;
                locked           = true;
                candidate_streak = confirm_frames;
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
                candidate_streak   = std::min(candidate_streak + 1, 1024);
                return;
            }

            if (candidate_order_idx == last_candidate_idx) candidate_streak++;
            else {
                last_candidate_idx = candidate_order_idx;
                candidate_streak   = 1;
            }

            if (candidate_streak >= confirm_frames) {
                committed_idx      = candidate_order_idx;
                last_candidate_idx = candidate_order_idx;
                candidate_streak   = 0;
            }
        }
    };

    struct MatchSearchOptions {
        bool single_observation { false };
        OrderSearchMode order_mode { OrderSearchMode::AllOrders };
        int continuity_spin_sign { +1 };
        int slot_count { 0 };
    };

    auto reset() -> void {
        order_state.reset();
        spin_state.reset();
        last_visible_slot_idx         = kUnknownOutpostSlotIdx;
        pre_predict_x_                = EKF::XVec::Zero();
        last_predict_dt_s_            = 0.0;
        has_predict_context           = false;
    }

    auto initialize(EKF::XVec& x) -> void {
        reset();
        OutpostState::apply_constraints(x);
    }

    auto current_order_idx() const -> int {
        return order_state.current_or_default(neutral_outpost_order_for_slot(0));
    }

    auto current_spin_sign() const -> int { return spin_state.current_sign(); }

    auto spin_locked() const -> bool { return spin_state.locked; }

    static auto normalize_spin_sign(int spin_sign) -> int { return SpinState::normalize(spin_sign); }

    auto locked_spin_sign() const -> std::optional<int> { return spin_state.locked_sign(); }

    auto record_predict_context(EKF::XVec const& x, double dt_s) -> void {
        pre_predict_x_      = x;
        last_predict_dt_s_  = dt_s;
        has_predict_context = true;
    }

    auto correct(EKF::XVec& x) const -> void { x[5] = util::normalize_angle(x[5]); }

    auto build_hypothesis_state(EKF::XVec const& fallback_x, int spin_sign) const -> EKF::XVec {
        auto x_test = has_predict_context ? pre_predict_x_ : fallback_x;
        if (has_predict_context && last_predict_dt_s_ > 0.0) {
            x_test = OutpostEKFParameters::f(last_predict_dt_s_, spin_sign)(x_test);
        } else {
            OutpostState::apply_constraints(x_test);
        }
        return x_test;
    }

    static auto limit_outpost_armors(std::span<Armor3D const> armors) -> std::span<Armor3D const> {
        return armors.first(
            std::min<std::size_t>(armors.size(), OutpostEKFParameters::kOutpostArmorCount));
    }

    auto build_observations(std::span<Armor3D const> armors) const
        -> std::vector<OutpostObservation> {
        auto limited_armors = limit_outpost_armors(armors);
        auto observations = std::vector<OutpostObservation> { };
        observations.reserve(limited_armors.size());
        for (auto const& armor : limited_armors)
            observations.emplace_back(outpost_observation(armor));
        return observations;
    }

    auto build_match_search_options(std::vector<OutpostObservation> const& observations,
        OutpostState::MatchContext const& context, std::optional<int> forced_spin_sign) const
        -> MatchSearchOptions {
        auto const single_observation = observations.size() == 1;
        auto const order_mode         = !single_observation   ? OrderSearchMode::AllOrders
            : has_committed_order()                        ? OrderSearchMode::CommittedOrderOnly
                                                           : OrderSearchMode::NeutralHeightOnly;
        auto const continuity_spin_sign =
            normalize_spin_sign(forced_spin_sign.value_or(current_spin_sign()));
        auto const slot_count = std::min(context.armor_num, OutpostEKFParameters::kOutpostArmorCount);
        return { single_observation, order_mode, continuity_spin_sign, slot_count };
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

    static auto estimate_center_z(std::vector<OutpostObservation> const& observations,
        std::vector<int> const& assignment, int order_idx, double armor_height_step) -> double {
        double z_sum = 0.0;
        for (std::size_t i = 0; i < observations.size(); ++i) {
            auto slot = assignment[i];
            z_sum += observations[i].xyz[2]
                - OutpostEKFParameters::outpost_height_rank(order_idx, slot) * armor_height_step;
        }
        return z_sum / static_cast<double>(observations.size());
    }

    auto build_assignment_state(std::vector<OutpostObservation> const& observations,
        std::vector<int> const& assignment, OutpostState::MatchContext const& context,
        MatchSearchOptions const& options, int order_idx, int spin_sign) const -> EKF::XVec {
        auto x_test = build_hypothesis_state(context.x, spin_sign);
        if (!options.single_observation)
            x_test[4] =
                estimate_center_z(observations, assignment, order_idx, kOutpostArmorHeightStep);
        return x_test;
    }

    auto observation_cost(OutpostObservation const& observation,
        OutpostState::MatchContext const& context, EKF::XVec const& x_test,
        MatchSearchOptions const& options, int slot, int order_idx) const
        -> std::optional<double> {
        auto xyz_pred = OutpostEKFParameters::h_armor_xyz(x_test, slot, order_idx);
        auto ypd_pred      = util::xyz2ypd(xyz_pred);
        auto yaw_pred      = OutpostEKFParameters::armor_yaw(x_test, slot);
        auto yaw_error     = std::abs(util::normalize_angle(observation.ypr[0] - yaw_pred));
        auto azimuth_error = std::abs(util::normalize_angle(observation.ypd[0] - ypd_pred[0]));
        auto z_error       = std::abs(observation.xyz[2] - xyz_pred[2]);
        auto const z_gate =
            options.single_observation ? outpost_single_observation_z_gate : outpost_z_gate;
        if (yaw_error > outpost_yaw_gate || azimuth_error > outpost_azimuth_gate
            || z_error > z_gate)
            return std::nullopt;

        double cost = 0.0;
        if (options.single_observation) {
            auto yaw_cost     = yaw_error / outpost_yaw_gate;
            auto azimuth_cost = azimuth_error / outpost_azimuth_gate;
            auto z_cost       = z_error / outpost_single_observation_z_gate;
            cost              = yaw_cost * yaw_cost + azimuth_cost * azimuth_cost
                + outpost_single_observation_z_weight * z_cost * z_cost;
        } else {
            auto H          = OutpostEKFParameters::H(x_test, slot, order_idx);
            auto z_hat      = OutpostEKFParameters::h(x_test, slot, order_idx);
            auto innovation = OutpostEKFParameters::z_subtract(observation.z, z_hat);
            auto S          = H * context.P * H.transpose() + observation.R;
            auto solved     = S.ldlt().solve(innovation);
            cost            = innovation.dot(solved);
        }

        if (!std::isfinite(cost)) return std::nullopt;
        return cost;
    }

    auto assignment_cost(std::vector<OutpostObservation> const& observations,
        std::vector<int> const& assignment, OutpostState::MatchContext const& context,
        MatchSearchOptions const& options, int order_idx, int spin_sign) const
        -> std::optional<double> {
        auto x_test = build_assignment_state(
            observations, assignment, context, options, order_idx, spin_sign);

        double total_cost = 0.0;
        for (std::size_t i = 0; i < observations.size(); ++i) {
            auto cost = observation_cost(
                observations[i], context, x_test, options, assignment[i], order_idx);
            if (!cost.has_value()) return std::nullopt;
            total_cost += cost.value();
        }
        return total_cost;
    }

    auto update_best_match(std::vector<OutpostObservation> const& observations,
        std::vector<int> const& assignment, OutpostState::MatchContext const& context,
        MatchSearchOptions const& options, int order_idx, int spin_sign,
        OutpostState::FrameMatch& best) const -> void {
        auto total_cost =
            assignment_cost(observations, assignment, context, options, order_idx, spin_sign);
        if (!total_cost.has_value() || total_cost.value() >= best.cost) return;

        best.valid                = true;
        best.order_idx            = order_idx;
        best.spin_sign            = spin_sign;
        best.cost                 = total_cost.value();
        best.visible_slot_idx     = select_visible_slot(observations, assignment);
        best.slots_by_observation = assignment;
    }

    auto search_slot_assignments(std::size_t obs_index,
        std::vector<OutpostObservation> const& observations,
        OutpostState::MatchContext const& context, MatchSearchOptions const& options,
        int order_idx, int spin_sign, std::vector<int>& assignment,
        std::array<bool, OutpostEKFParameters::kOutpostArmorCount>& slot_used,
        OutpostState::FrameMatch& best) const -> void {
        if (obs_index == observations.size()) {
            update_best_match(
                observations, assignment, context, options, order_idx, spin_sign, best);
            return;
        }

        for (int slot = 0; slot < options.slot_count; ++slot) {
            if (slot_used[slot]) continue;
            if (!is_slot_assignment_feasible(slot, order_idx, options)) continue;
            slot_used[slot]       = true;
            assignment[obs_index] = slot;
            search_slot_assignments(obs_index + 1, observations, context, options, order_idx,
                spin_sign, assignment, slot_used, best);
            slot_used[slot]       = false;
            assignment[obs_index] = -1;
        }
    }

    auto max_match_cost(MatchSearchOptions const& options) const -> double {
        return options.single_observation ? outpost_single_observation_cost_gate
                                          : outpost_mahalanobis_gate;
    }

    auto match_armor(Armor3D const& armor, OutpostState::MatchContext const& context) const
        -> OutpostState::MatchResult {
        auto forced_spin_sign = locked_spin_sign();
        auto frame_match =
            match_frame(std::span<Armor3D const> { &armor, 1 }, context, forced_spin_sign);
        if (!frame_match.valid) return { -1, 1e10, false };
        return { frame_match.slots_by_observation.front(), frame_match.cost, true };
    }

    auto match_frame(std::span<Armor3D const> armors, OutpostState::MatchContext const& context,
        std::optional<int> forced_spin_sign) const -> OutpostState::FrameMatch {
        if (armors.empty()) return { };

        auto observations = build_observations(armors);

        auto best = OutpostState::FrameMatch { };
        best.cost = std::numeric_limits<double>::infinity();

        auto const options = build_match_search_options(observations, context, forced_spin_sign);
        auto assignment    = std::vector<int>(observations.size(), -1);
        auto slot_used     = std::array<bool, OutpostEKFParameters::kOutpostArmorCount> { };

        auto visit_order_idx = [&](int order_idx) {
            if (forced_spin_sign.has_value()) {
                auto spin_sign = normalize_spin_sign(forced_spin_sign.value());
                search_slot_assignments(0, observations, context, options, order_idx, spin_sign,
                    assignment, slot_used, best);
                return;
            }

            for (int spin_sign : { -1, +1 })
                search_slot_assignments(0, observations, context, options, order_idx, spin_sign,
                    assignment, slot_used, best);
        };

        if (options.order_mode == OrderSearchMode::CommittedOrderOnly) {
            visit_order_idx(current_order_idx());
        } else {
            for (int order_idx = 0; order_idx < OutpostEKFParameters::kOutpostHeightOrderCount;
                ++order_idx)
                visit_order_idx(order_idx);
        }

        if (!best.valid || best.cost > max_match_cost(options)) return { };
        return best;
    }

    auto update(EKF& ekf, std::span<Armor3D const> armors, int armor_num, int& update_count)
        -> bool {
        if (armors.empty()) return false;

        auto used_armors       = limit_outpost_armors(armors);
        auto forced_spin_sign  = locked_spin_sign();
        auto frame_match = match_frame(used_armors,
            OutpostState::MatchContext { ekf.x, ekf.P(), armor_num }, forced_spin_sign);
        if (!frame_match.valid) return false;

        update_count++;

        auto const allow_order_recalibration = used_armors.size() >= 2;
        ekf.x                                = build_hypothesis_state(ekf.x, frame_match.spin_sign);
        OutpostState::apply_constraints(ekf.x);

        auto observation_order = std::vector<std::size_t>(used_armors.size());
        std::iota(observation_order.begin(), observation_order.end(), std::size_t { 0 });
        std::sort(observation_order.begin(), observation_order.end(),
            [&](std::size_t lhs, std::size_t rhs) {
                return frame_match.slots_by_observation[lhs]
                    < frame_match.slots_by_observation[rhs];
            });

        for (auto obs_index : observation_order) {
            auto observation = outpost_observation(used_armors[obs_index]);
            auto slot_idx    = frame_match.slots_by_observation[obs_index];

            ekf.update(
                observation.z,
                [slot_idx, order_idx = frame_match.order_idx](EKF::XVec const& x) {
                    return OutpostEKFParameters::h(x, slot_idx, order_idx);
                },
                [slot_idx, order_idx = frame_match.order_idx](EKF::XVec const& x) {
                    return OutpostEKFParameters::H(x, slot_idx, order_idx);
                },
                observation.R, OutpostEKFParameters::x_add, OutpostEKFParameters::z_subtract);

            OutpostState::apply_constraints(ekf.x);
        }

        apply_frame_match(frame_match, allow_order_recalibration);
        return true;
    }

    auto apply_frame_match(
        OutpostState::FrameMatch const& frame_match, bool allow_order_recalibration) -> void {
        if (!frame_match.valid) return;

        spin_state.accept_frame_sign(frame_match.spin_sign, outpost_spin_confirm_frames);
        last_visible_slot_idx = frame_match.visible_slot_idx;
        if (allow_order_recalibration)
            order_state.accept_candidate(frame_match.order_idx, outpost_order_confirm_frames);
    }

    auto is_converged(EKF::XVec const&, EKF::PMat const& P, int update_count) const -> bool {
        auto const xy_var_ok = P(0, 0) < 0.1 && P(2, 2) < 0.1;
        auto const z_var_ok  = P(4, 4) < 0.05;
        auto const yaw_var_ok = P(5, 5) < 0.2;
        return spin_locked() && xy_var_ok && z_var_ok && yaw_var_ok && update_count > 10;
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
        auto normalized_slot =
            (slot_idx % OutpostEKFParameters::kOutpostArmorCount
                + OutpostEKFParameters::kOutpostArmorCount)
            % OutpostEKFParameters::kOutpostArmorCount;
        auto step = spin_sign >= 0 ? -1 : +1;
        return (normalized_slot + step + OutpostEKFParameters::kOutpostArmorCount)
            % OutpostEKFParameters::kOutpostArmorCount;
    }

    static auto select_visible_slot(std::vector<OutpostObservation> const& observations,
        std::vector<int> const& assignment) -> int {
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

        auto z = EKF::ZVec { };
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

    OrderState order_state { };
    SpinState spin_state { };
    int last_visible_slot_idx { kUnknownOutpostSlotIdx };

    EKF::XVec pre_predict_x_ { EKF::XVec::Zero() };
    double last_predict_dt_s_ { 0.0 };
    bool has_predict_context { false };
};

OutpostState::OutpostState() noexcept
    : pimpl { std::make_unique<Impl>() } { }
OutpostState::~OutpostState() noexcept = default;

auto OutpostState::reset() -> void { return pimpl->reset(); }

auto OutpostState::initialize(EKF::XVec& x) -> void { return pimpl->initialize(x); }

auto OutpostState::current_order_idx() const -> int { return pimpl->current_order_idx(); }

auto OutpostState::current_spin_sign() const -> int { return pimpl->current_spin_sign(); }

auto OutpostState::spin_locked() const -> bool { return pimpl->spin_locked(); }

auto OutpostState::record_predict_context(EKF::XVec const& x, double dt_s) -> void {
    return pimpl->record_predict_context(x, dt_s);
}

auto OutpostState::correct(EKF::XVec& x) const -> void { return pimpl->correct(x); }

auto OutpostState::build_hypothesis_state(EKF::XVec const& fallback_x, int spin_sign) const
    -> EKF::XVec {
    return pimpl->build_hypothesis_state(fallback_x, spin_sign);
}

auto OutpostState::match_armor(Armor3D const& armor, MatchContext const& context) const
    -> MatchResult {
    return pimpl->match_armor(armor, context);
}

auto OutpostState::match_frame(std::span<Armor3D const> armors, MatchContext const& context,
    std::optional<int> forced_spin_sign) const -> FrameMatch {
    return pimpl->match_frame(armors, context, forced_spin_sign);
}

auto OutpostState::update(
    EKF& ekf, std::span<Armor3D const> armors, int armor_num, int& update_count) -> bool {
    return pimpl->update(ekf, armors, armor_num, update_count);
}

auto OutpostState::apply_frame_match(FrameMatch const& frame_match, bool allow_order_recalibration)
    -> void {
    return pimpl->apply_frame_match(frame_match, allow_order_recalibration);
}

auto OutpostState::is_converged(EKF::XVec const& x, EKF::PMat const& P, int update_count) const
    -> bool {
    return pimpl->is_converged(x, P, update_count);
}

auto OutpostState::apply_constraints(EKF::XVec& x) -> void { x[5] = util::normalize_angle(x[5]); }
