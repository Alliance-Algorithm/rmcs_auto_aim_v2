#pragma once
#include <chrono>

namespace rmcs::util {
constexpr auto delta_time(std::chrono::steady_clock::time_point const& late,
    std::chrono::steady_clock::time_point const& early) -> auto {
    return std::chrono::duration<double>(late - early);
}

}
