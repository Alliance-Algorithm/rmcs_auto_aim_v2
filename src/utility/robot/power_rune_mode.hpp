#pragma once

#include <cstdint>
#include <optional>

namespace rmcs {
enum class PowerRuneMode : std::uint8_t { LARGE, SMALL };
// nullopt means rune mode is not activated/known yet.
using PowerRuneModeOpt = std::optional<PowerRuneMode>;
} // namespace rmcs

