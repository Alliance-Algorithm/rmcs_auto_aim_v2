#pragma once

#include "utility/image/image.hpp"
#include "utility/pimpl.hpp"

#include <expected>
#include <optional>

#include <opencv2/core/types.hpp>
#include <yaml-cpp/yaml.h>

namespace rmcs::identifier {

class GreenLightDetection {
    RMCS_PIMPL_DEFINITION(GreenLightDetection)

public:
    auto initialize(const YAML::Node&) noexcept -> std::expected<void, std::string>;
    auto sync_detect(const Image&, const cv::Rect2i&) noexcept -> std::optional<cv::Rect2i>;
    auto green_light() const noexcept -> std::optional<cv::Rect2i>;
};

}
