#pragma once

#include <chrono>

namespace rmcs::util {

constexpr auto delta_time(std::chrono::steady_clock::time_point late,
    std::chrono::steady_clock::time_point early) -> auto {
    return std::chrono::duration<double>(late - early);
}

}