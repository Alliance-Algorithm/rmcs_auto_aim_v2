#pragma once
#include "utility/robot/armor.hpp"

#include <opencv2/core/mat.hpp>

namespace rmcs::util {

auto optimize_corners(const cv::Mat& image, Armor2ds& armors) -> void;

auto optimize_corners(const cv::Mat& image, Lightbar2d& lightbar) -> void;

}
