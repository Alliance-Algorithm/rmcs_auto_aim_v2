#pragma once

#include <cmath>
namespace rmcs::util {
inline auto normalize_angle(double angle) -> auto {
    return std::atan2(std::sin(angle), std::cos(angle));
}

}
