#pragma once

#include <eigen3/Eigen/Core>

#include "module/fire_control/strategy/fire_control_adapter.hpp"
#include "module/predictor/snapshot.hpp"
#include "utility/math/angle.hpp"

namespace rmcs::fire_control {

struct FireGateContext {
    predictor::Snapshot const& snapshot;
    TimePoint sample_time;
    Eigen::Vector3d const& aim_position;
    bool control { false };
};

struct FireGateResult {
    bool allowed { false };
};

class FireGateEvaluator {
public:
    auto evaluate(GateMode mode, FireGateContext const& context) const -> FireGateResult;

private:
    const double kFacingYawThresholdRad { util::deg2rad(25.0) };
};

} // namespace rmcs::fire_control
