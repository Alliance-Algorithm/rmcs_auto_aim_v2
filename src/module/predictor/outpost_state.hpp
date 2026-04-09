#pragma once

#include <cstddef>
#include <optional>
#include <span>
#include <vector>

#include "module/predictor/outpost_ekf_parameter.hpp"
#include "utility/pimpl.hpp"

namespace rmcs::predictor {
class OutpostState {
    RMCS_PIMPL_DEFINITION(OutpostState)
public:
    using EKF = OutpostEKFParameters::EKF;

    struct MatchContext {
        EKF::XVec const& x;
        EKF::PMat const& P;
        int armor_num;
    };

    struct FrameMatch {
        bool valid { false };
        int order_idx { 0 };
        int spin_sign { +1 };
        double cost { 1e10 };
        int visible_slot_idx { -1 };
        std::vector<int> slots_by_observation;
    };

    struct MatchResult {
        int armor_id;
        double error;
        bool is_valid;
    };

    auto reset() -> void;
    auto initialize(EKF::XVec& x) -> void;

    auto current_order_idx() const -> int;
    auto current_spin_sign() const -> int;
    auto spin_locked() const -> bool;

    auto record_predict_context(EKF::XVec const& x, double dt_s) -> void;
    auto correct(EKF::XVec& x) const -> void;

    auto build_hypothesis_state(EKF::XVec const& fallback_x, int spin_sign) const -> EKF::XVec;

    auto match_armor(Armor3D const& armor, MatchContext const& context) const -> MatchResult;
    auto match_frame(std::span<Armor3D const> armors, MatchContext const& context,
        std::optional<int> forced_spin_sign = std::nullopt) const -> FrameMatch;
    auto update(EKF& ekf, std::span<Armor3D const> armors, int armor_num, int& update_count)
        -> bool;
    auto apply_frame_match(FrameMatch const& frame_match, bool allow_order_recalibration) -> void;

    auto is_converged(EKF::XVec const& x, EKF::PMat const& P, int update_count) const -> bool;

    static auto apply_constraints(EKF::XVec& x) -> void;
};
}
