#pragma once
#include "utility/robot/color.hpp"

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

namespace rmcs {

struct LightbarFinder {
    struct Input {
        cv::Mat roi;
        CampColor color { CampColor::UNKNOWN };
        cv::Point2i predicted_point1;
        cv::Point2i predicted_point2;
    } input;

    struct Result {
        cv::Point2i point1;
        cv::Point2i point2;
    } result;

    auto solve() -> bool;
};

}
