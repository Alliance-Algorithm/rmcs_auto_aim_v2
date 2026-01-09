#pragma once

#include "utility/robot/id.hpp"
#include <unordered_map>

namespace rmcs {

using PriorityMode = std::unordered_map<DeviceId, int>;

}
