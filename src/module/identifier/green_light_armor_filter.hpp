#pragma once

#include "utility/image/image.hpp"
#include "utility/pimpl.hpp"
#include "utility/robot/armor.hpp"

#include <expected>
#include <optional>
#include <span>
#include <vector>

#include <opencv2/core/types.hpp>

#include <yaml-cpp/yaml.h>

namespace rmcs::identifier {

class GreenLightArmorFilter {
    RMCS_PIMPL_DEFINITION(GreenLightArmorFilter)

public:
    auto initialize(const YAML::Node&) noexcept -> std::expected<void, std::string>;
    auto filter(const Image&, std::span<const Armor2D>) noexcept -> std::vector<bool>;
    auto green_light() const noexcept -> std::optional<cv::Rect2i>;
};

}
