#pragma once

#include <array>

namespace rmcs::predictor {

struct OutpostArmorSlot {
    double phase_offset { 0.0 };
    double height_offset { 0.0 };
    bool assigned { false };
};

struct OutpostArmorLayout {
    static constexpr int kSlotCount = 3;

    std::array<OutpostArmorSlot, kSlotCount> slots {};
};

} // namespace rmcs::predictor
