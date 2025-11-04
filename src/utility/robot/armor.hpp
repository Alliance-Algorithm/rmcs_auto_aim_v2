#pragma once
#include "utility/robot/id.hpp"

namespace rmcs {

enum class ArmorColor : std::uint8_t { DARK, RED, BLUE, MIX };
constexpr auto get_enum_name(ArmorColor color) noexcept {
    constexpr std::array details { "dark", "red", "blue", "mix" };
    return details[std::to_underlying(color)];
}

enum class ArmorShape : bool { LARGE, SMALL };
constexpr auto get_enum_name(ArmorShape shape) noexcept {
    constexpr std::array details { "large", "small" };
    return details[std::to_underlying(shape)];
};

using ArmorType = DeviceId;
constexpr auto get_enum_name(ArmorType type) noexcept { return to_string(type); }

struct LightStrip { };

struct Armor { };

}
