#pragma once

#include <cstdint>
#include <string_view>

namespace rmcs {

enum class CampColor : uint8_t {
    UNKNOWN,
    RED,
    BLUE,
};

constexpr auto to_string(CampColor color) noexcept -> std::string_view {
    switch (color) {
    case CampColor::UNKNOWN:
        return "UNKNOWN";
    case CampColor::RED:
        return "RED";
    case CampColor::BLUE:
        return "BLUE";
    }
    return "UNKNOWN";
}
}
