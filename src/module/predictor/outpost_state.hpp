#pragma once

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
        EKF::XVec x;
        EKF::PMat P;
        EKF::XVec negative_spin_x;
        EKF::XVec positive_spin_x;
    };

    struct MatchResult {
        int armor_id;
        double error;
        bool is_valid;
    };

    struct FrameMatch {
        bool valid { false };
        int order_idx { 0 };
        int spin_sign { +1 };
        double cost { 1e10 };
        int visible_slot_idx { -1 };
        std::vector<int> slots_by_observation;
    };

    auto reset() -> void;

    auto current_order_idx() const -> int;
    auto current_spin_sign() const -> int;
    auto spin_locked() const -> bool;

    auto match_armor(Armor3D const& armor, MatchContext const& context) const -> MatchResult;
    auto match_frame(std::span<Armor3D const> armors, MatchContext const& context) const
        -> FrameMatch;
    auto apply_frame_match(FrameMatch const& frame_match) -> void;
};
}
