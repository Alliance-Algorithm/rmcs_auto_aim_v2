#pragma once

#include <chrono>

#include "module/predictor/snapshot.hpp"
#include "utility/pimpl.hpp"

namespace rmcs::predictor {
struct RobotState {
    RMCS_PIMPL_DEFINITION(RobotState)
public:
    struct MatchResult {
        int armor_id;
        double error;
        bool is_valid;
    };

    auto initialize(Armor3D const&, std::chrono::steady_clock::time_point const&) -> void;

    auto predict(std::chrono::steady_clock::time_point const& t) -> void;

    auto match(Armor3D const& armor) const -> MatchResult;
    auto update(Armor3D const& armor) -> void;

    auto is_convergened() const -> bool;

    auto get_snapshot() const -> Snapshot;

    auto distance() const -> double;
};
}
