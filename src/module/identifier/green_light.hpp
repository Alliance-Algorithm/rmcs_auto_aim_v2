#pragma once

#include "utility/image/image.hpp"
#include "utility/pimpl.hpp"
#include "utility/robot/armor.hpp"

#include <expected>
#include <optional>
#include <span>

#include <opencv2/core/types.hpp>
#include <yaml-cpp/yaml.h>

namespace rmcs::identifier {

class GreenLightFinder {
    RMCS_PIMPL_DEFINITION(GreenLightFinder)

public:
    struct Result {
        std::optional<cv::Rect2i> detect_roi  = std::nullopt;
        std::optional<cv::Rect2i> green_light = std::nullopt;
    };

    auto initialize(const YAML::Node&) noexcept -> std::expected<void, std::string>;
    auto locate(const Image&, std::span<const Armor2d>) noexcept -> Result;
};

}
