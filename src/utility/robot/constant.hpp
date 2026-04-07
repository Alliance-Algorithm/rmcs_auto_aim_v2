#pragma once

#include <numbers>

namespace rmcs {
constexpr double kBaseRadius    = 0.3205;
constexpr double kOutpostRadius = 0.275;
constexpr double kOtherRadius   = 0.2;

constexpr double kPredictedOutpostArmorPitch = -15. / 180. * std::numbers::pi;
constexpr double kPredictedOtherArmorPitch   = 15. / 180. * std::numbers::pi;

constexpr double kOutpostArmorHeightStep = 0.102;
constexpr double kOutpostAngularSpeed    = 0.8 * std::numbers::pi;

}
