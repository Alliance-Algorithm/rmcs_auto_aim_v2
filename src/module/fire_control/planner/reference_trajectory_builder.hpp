#pragma once

#include <expected>
#include <functional>
#include <string>

#include "module/fire_control/planner/mpc_types.hpp"
#include "module/fire_control/solver/aim_point_sampling.hpp"
#include "utility/clock.hpp"

namespace rmcs::fire_control {

class ReferenceTrajectoryBuilder {
public:
    using AimAttitudeSampler = std::function<std::expected<AimAttitude, std::string>(TimePoint)>;

    auto build(TimePoint center_time, AimAttitudeSampler const& sample_attitude) const
        -> std::expected<ReferenceTrajectory, std::string>;

private:
    static constexpr double kMpcAxisDt = 0.01;
};

} // namespace rmcs::fire_control
