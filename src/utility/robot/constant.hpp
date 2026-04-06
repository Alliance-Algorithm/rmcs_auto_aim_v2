#pragma once

#include <array>
#include <numbers>

namespace rmcs {
constexpr double kBaseRadius    = 0.3205;
constexpr double kOutpostRadius = 0.275;
constexpr double kOtherRadius   = 0.2;

constexpr double kOutpostArmorHeightStep = 0.102;
constexpr double kPredictedArmorPitch    = -15. / 180. * std::numbers::pi;
constexpr double kOutpostAngularSpeed    = 0.8 * std::numbers::pi;
constexpr int kOutpostArmorCount         = 3;
constexpr int kOutpostHeightOrderCount   = 6;
constexpr std::array<std::array<int, kOutpostArmorCount>, kOutpostHeightOrderCount>
    kOutpostHeightOrders { {
        { -1, 0, +1 },
        { 0, +1, -1 },
        { +1, -1, 0 },
        { -1, +1, 0 },
        { +1, 0, -1 },
        { 0, -1, +1 },
    } };

}
