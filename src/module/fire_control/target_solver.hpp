#pragma once

#include <expected>
#include <string>
#include <vector>

#include "module/fire_control/types.hpp"
#include "module/predictor/snapshot.hpp"
#include "utility/pimpl.hpp"

namespace rmcs::fire_control {

class TargetSolver {
    RMCS_PIMPL_DEFINITION(TargetSolver)

public:
    auto make_candidates(predictor::Snapshot const& snapshot, TimePoint sample_time) const
        -> std::vector<ArmorCandidate>;

    auto solve(predictor::Snapshot const& snapshot, int armor_id, TimePoint command_time,
        double bullet_speed, double shoot_delay) const -> std::expected<TargetSolution, std::string>;
};

} // namespace rmcs::fire_control
