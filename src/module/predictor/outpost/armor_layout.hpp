#pragma once

#include <array>
#include <optional>

namespace rmcs::predictor {

struct OutpostArmorLayout {
    static constexpr int kSlotCount = 3;

    std::array<std::optional<int>, kSlotCount> height_levels {};
};

} // namespace rmcs::predictor
