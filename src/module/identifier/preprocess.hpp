#pragma once

#include <opencv2/core/mat.hpp>

namespace rmcs {

struct PreProcess {

    double binarization_threshold;

    auto process(const cv::Mat&, cv::Mat&) const noexcept -> void;
};

}
