#pragma once

#include <unordered_map>

#include "utility/robot/id.hpp"
namespace rmcs {
using PriorityMode = std::unordered_map<DeviceId, int>;
}
