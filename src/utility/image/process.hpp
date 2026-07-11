#pragma once

#include "utility/robot/color.hpp"
#include <opencv2/core/mat.hpp>

namespace rmcs::util {

auto extract_channel(const cv::Mat& src, CampColor camp, cv::Mat& dst) -> void;

}
