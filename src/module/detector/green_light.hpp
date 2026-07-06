#pragma once

#include "utility/pimpl.hpp"
#include "utility/robot/armor.hpp"

#include <expected>
#include <optional>
#include <span>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <yaml-cpp/yaml.h>

namespace rmcs::detector {

class GreenLightFinder {
    RMCS_PIMPL_DEFINITION(GreenLightFinder)

public:
    struct Result {
        std::optional<cv::Rect2i> detect_roi  = std::nullopt;
        std::optional<cv::Rect2i> green_light = std::nullopt;
    };

    auto initialize(const YAML::Node&) noexcept -> std::expected<void, std::string>;
    auto locate(const cv::Mat&, std::span<const Armor2d>) noexcept -> Result;
};

}
