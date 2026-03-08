#pragma once

#include <numbers>
namespace rmcs {

constexpr double kBaseRadius    = 0.3205;
constexpr double kOutpostRadius = 0.2765;
constexpr double kRuneRadius    = 0.7;
constexpr double kOtherRadius   = 0.2;

constexpr double kSmallRuneAngularVelocity = 1. / 3. * std::numbers::pi;
constexpr double kLargeRuneAmplitudeMin    = 0.780;
constexpr double kLargeRuneAmplitudeMax    = 1.045;
constexpr double kLargeRuneOmegaMin        = 1.884;
constexpr double kLargeRuneOmegaMax        = 2.000;
constexpr double kLargeRuneOffsetBase      = 2.090;

}
