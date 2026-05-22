#pragma once

#include "utility/robot/id.hpp"

#include <cstdint>
#include <string_view>

namespace rmcs {

enum class TargetMode : uint8_t {
    COMBAT = 0,
    OUTPOST_ONLY,
};

constexpr auto to_string(TargetMode mode) noexcept -> std::string_view {
    switch (mode) {
    case TargetMode::COMBAT: return "COMBAT";
    case TargetMode::OUTPOST_ONLY: return "OUTPOST_ONLY";
    }

    return {};
}

constexpr auto allowed_target_ids(TargetMode mode) noexcept -> DeviceIds {
    switch (mode) {
    case TargetMode::COMBAT: return DeviceIds::kGround();
    case TargetMode::OUTPOST_ONLY: return DeviceIds { DeviceId::OUTPOST };
    }

    return DeviceIds::None();
}

constexpr auto target_mode_allows(TargetMode mode, DeviceId device_id) noexcept -> bool {
    return allowed_target_ids(mode).contains(device_id);
}

} // namespace rmcs
