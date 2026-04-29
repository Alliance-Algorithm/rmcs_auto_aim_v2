#include "module/fire_control/strategy/fire_gate_evaluator.hpp"

#include <cmath>

#include <eigen3/Eigen/Geometry>

#include "utility/math/conversion.hpp"

using namespace rmcs::fire_control;

auto FireGateEvaluator::evaluate(GateMode mode, FireGateContext const& context) const
    -> FireGateResult {
    if (mode == GateMode::DEFAULT) {
        return FireGateResult { .allowed = true };
    }

    auto predicted_armors = context.snapshot.predicted_armors(context.sample_time);
    for (auto const& armor : predicted_armors) {
        auto armor_position = Eigen::Vector3d {};
        armor.translation.copy_to(armor_position);
        auto const yaw_to_armor = std::atan2(armor_position.y(), armor_position.x());

        auto orientation = Eigen::Quaterniond {};
        armor.orientation.copy_to(orientation);
        auto const armor_facing_yaw = util::eulers(orientation)[0];

        auto const yaw_error = std::abs(util::normalize_angle(armor_facing_yaw - yaw_to_armor));
        if (yaw_error < kFacingYawThresholdRad) {
            return FireGateResult { .allowed = true };
        }
    }

    return FireGateResult { .allowed = false };
}
