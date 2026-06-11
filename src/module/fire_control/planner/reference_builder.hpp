#pragma once

#include <expected>
#include <string>

#include "module/fire_control/planner/mpc_types.hpp"
#include "module/fire_control/types.hpp"
#include "module/predictor/snapshot.hpp"
#include "module/fire_control/target_solver.hpp"
#include "utility/pimpl.hpp"

namespace rmcs::fire_control {

class ReferenceBuilder {
    RMCS_PIMPL_DEFINITION(ReferenceBuilder)

public:
    auto build(predictor::Snapshot const& snapshot, TargetSolution const& raw_solution,
        TimePoint command_time, double bullet_speed, double shoot_delay)
        -> std::expected<ReferenceTrajectory, std::string>;
};

} // namespace rmcs::fire_control
