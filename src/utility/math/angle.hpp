#pragma once

#include <cmath>
#include <numbers>

namespace rmcs::util {

inline auto normalize_angle(double angle) -> double {
    return std::atan2(std::sin(angle), std::cos(angle));
}

constexpr auto deg2rad(double deg) -> double { return deg * std::numbers::pi / 180.0; }
constexpr auto rad2deg(double rad) -> double { return rad * 180.0 / std::numbers::pi; }

}