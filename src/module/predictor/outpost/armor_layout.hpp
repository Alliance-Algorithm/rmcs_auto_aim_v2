#pragma once

#include <array>

namespace rmcs::predictor {

struct OutpostArmorSlot {
    double phase_offset { 0.0 };
    double height_offset { 0.0 };
    bool assigned { false };
};

struct OutpostArmorLayout {
    std::array<OutpostArmorSlot, 3> slots {};
};

} // namespace rmcs::predictor
