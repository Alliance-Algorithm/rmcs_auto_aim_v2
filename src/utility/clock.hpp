#pragma once
#include <chrono>

namespace rmcs {

using Clock     = std::chrono::steady_clock;
using TimePoint = Clock::time_point;
using Duration  = TimePoint::duration;
using Timestamp = TimePoint;

} // namespace rmcs
