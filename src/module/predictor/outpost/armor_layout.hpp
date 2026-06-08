#pragma once

#include <optional>

namespace rmcs::predictor {

struct OutpostArmorLayout {
    enum class HeightTemplate { PositiveOnSlot1, NegativeOnSlot1 };
    std::optional<HeightTemplate> height_template;
};

} // namespace rmcs::predictor
