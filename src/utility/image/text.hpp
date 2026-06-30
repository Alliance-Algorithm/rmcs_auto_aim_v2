#pragma once

#include <string>

#include <opencv2/core/mat.hpp>

namespace rmcs::util {

auto draw_text(cv::Mat&, const std::string& text) noexcept -> void;

}
