#include "preprocess.hpp"

#include <opencv2/imgproc.hpp>

using namespace rmcs;

auto PreProcess::process(const cv::Mat& image, cv::Mat& out) const noexcept -> void {
    auto gray_image = cv::Mat {};
    cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);

    cv::threshold(gray_image, out, binarization_threshold, 255, cv::THRESH_BINARY);
}
