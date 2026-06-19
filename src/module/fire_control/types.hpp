#pragma once

#include "module/fire_control/gimbal_state.hpp"
#include "module/predictor/snapshot.hpp"
#include "utility/robot/armor.hpp"

namespace rmcs::fire_control {

struct AimAttitude {
    double yaw { 0.0 };
    double pitch { 0.0 };
    double fly_time { 0.0 };
};

struct ArmorCandidate {
    Armor3d armor;
    predictor::TargetMotion motion;
};

struct TargetSolution {
    TimePoint impact_time;
    ArmorCandidate candidate;
    AimAttitude attitude;
};

} // namespace rmcs::fire_control
