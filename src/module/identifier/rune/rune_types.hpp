#pragma once

#include <vector>

#include <opencv2/core/types.hpp>

namespace rmcs::identifier {

struct RuneCenterCandidate {
    int contour_index = -1;
    double area       = 0.0;
    cv::Point2f center;
    cv::RotatedRect rotated_rect;
};

struct RuneFeatureIdentifyResult {
    std::vector<RuneCenterCandidate> centers;
};

} // namespace rmcs::identifier
