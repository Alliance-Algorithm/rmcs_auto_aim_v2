#pragma once

#include "utility/clock.hpp"

namespace rmcs::fire_control {

struct GimbalState {
    TimePoint timestamp;
    double yaw { 0.0 };
    double pitch { 0.0 };
};

} // namespace rmcs::fire_control
