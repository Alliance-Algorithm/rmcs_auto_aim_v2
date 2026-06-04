#pragma once
#include "utility/robot/color.hpp"

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

namespace rmcs {

struct LightbarFinder {
    struct Input {
        cv::Mat source;
        CampColor color { CampColor::UNKNOWN };
        cv::Point2i predicted_upper;
        cv::Point2i predicted_lower;
    } input;

    struct Result {
        cv::Point2i upper;
        cv::Point2i lower;
    } result;

    auto solve() -> bool;
};

}
