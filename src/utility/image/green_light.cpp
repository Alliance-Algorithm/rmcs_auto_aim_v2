#include "green_light.hpp"

#include "utility/image/image.details.hpp"

#include <algorithm>

#include <opencv2/imgproc.hpp>

namespace rmcs::util {

auto draw_green_light(Image& canvas, const cv::Rect2i& rect) noexcept -> void {
    auto& opencv_mat = canvas.details().mat;

    cv::rectangle(opencv_mat, rect, cv::Scalar { 0, 255, 0 }, 2, cv::LINE_AA);
    cv::putText(opencv_mat, "green_light",
        cv::Point2i { rect.x, std::max(rect.y - 6, 0) }, cv::FONT_HERSHEY_SIMPLEX, 0.6,
        cv::Scalar { 0, 255, 0 }, 1, cv::LINE_AA);
}

auto draw_green_light_roi(Image& canvas, const cv::Rect2i& rect) noexcept -> void {
    auto& opencv_mat = canvas.details().mat;

    cv::rectangle(opencv_mat, rect, cv::Scalar { 0, 255, 255 }, 1, cv::LINE_AA);
    cv::putText(opencv_mat, "green_light_roi",
        cv::Point2i { rect.x, std::max(rect.y - 6, 0) }, cv::FONT_HERSHEY_SIMPLEX, 0.5,
        cv::Scalar { 0, 255, 255 }, 1, cv::LINE_AA);
}

}
