#pragma once

#include <cmath>
namespace rmcs::util {
inline auto normalize_angle(double angle) -> auto {
    return std::atan2(std::sin(angle), std::cos(angle));
}

inline constexpr auto deg2rad(double deg) -> double { return deg * M_PI / 180.0; }
inline constexpr auto rad2deg(double rad) -> double { return rad * 180.0 / M_PI; }

}
