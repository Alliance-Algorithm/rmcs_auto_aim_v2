#pragma once
#include "utility/robot/color.hpp"

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

#include <optional>

namespace rmcs {

struct LightbarFinder {
    struct Input {
        cv::Mat source;
        CampColor color;
        std::optional<cv::Point2i> predicted_upper;
        std::optional<cv::Point2i> predicted_lower;
    } input;

    struct Result {
        cv::Point2i upper;
        cv::Point2i lower;
    } result;

    auto solve() -> bool;
};

}
