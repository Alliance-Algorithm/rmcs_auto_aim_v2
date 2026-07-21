#pragma once

#include "utility/math/linear.hpp"
#include "utility/pimpl.hpp"

namespace rmcs {

class ShootEvaluator {
    RMCS_PIMPL_DEFINITION(ShootEvaluator)

public:
    struct Config {
        double yaw_tolerance { 0.07 };
        double pitch_tolerance { 0.04 };
        bool require_stable_command { true };
    };

    struct Command {
        double yaw   = kNaN;
        double pitch = kNaN;
        Point3d center;
        Point3d armor;
    };

    explicit ShootEvaluator(const Config& config);

    auto evaluate(Command const& command, double yaw, double pitch) noexcept -> bool;
};

} // namespace rmcs
