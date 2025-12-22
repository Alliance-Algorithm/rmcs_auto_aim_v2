#pragma once

#include <unordered_map>

#include "utility/robot/id.hpp"
namespace rmcs {
enum class RobotPriority {
    // clang-format off
    FIRST  = 1, 
    SECOND = 2,
    THIRD  = 3,
    FORTH  = 4,
    FIFTH  = 5
    // clang-format on
};

using PriorityMode = std::unordered_map<DeviceId, RobotPriority>;
}
