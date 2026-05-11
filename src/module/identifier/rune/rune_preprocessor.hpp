#pragma once

#include "utility/image/image.hpp"
#include "utility/pimpl.hpp"

#include <expected>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <yaml-cpp/yaml.h>

namespace rmcs::identifier {

struct RuneContourCandidate {
    std::vector<cv::Point> contour;
    double area = 0.0;
    cv::Rect bounding_rect;
    cv::RotatedRect rotated_rect;
    cv::Point2f center;
};

struct RunePreprocessResult {
    std::vector<RuneContourCandidate> candidates;
    std::vector<cv::Vec4i> hierarchy;
};

class RunePreprocessor {
    RMCS_PIMPL_DEFINITION(RunePreprocessor)

public:
    auto initialize(const YAML::Node&) noexcept -> std::expected<void, std::string>;

    auto sync_process(const Image&) noexcept -> std::optional<RunePreprocessResult>;
};

}
