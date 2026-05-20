#pragma once

#include "utility/image/image.hpp"

#include <opencv2/core/types.hpp>

namespace rmcs::util {

auto draw_green_light(Image&, const cv::Rect2i&) noexcept -> void;

}
