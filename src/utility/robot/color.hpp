#pragma once
#include <cstdint>

namespace rmcs {

enum class CampColor : uint8_t {
    UNKNOWN,
    RED,
    BLUE,
};

inline auto to_string(CampColor color) noexcept -> const char* {
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
