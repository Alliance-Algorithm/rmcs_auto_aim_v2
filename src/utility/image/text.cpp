#include "text.hpp"

#include "utility/image/image.details.hpp"

#include <algorithm>

#include <opencv2/imgproc.hpp>

namespace rmcs::util {

auto draw_text(Image& canvas, const std::string& text) noexcept -> void {
    auto& opencv_mat = canvas.details().mat;

    const auto thickness = 1;
    const auto font      = cv::FONT_HERSHEY_SIMPLEX;
    const auto scale     = 0.6;
    const auto margin    = 10;
    const auto white     = cv::Scalar { 255, 255, 255 };
    const auto black     = cv::Scalar { 0, 0, 0 };

    auto baseline = 0;
    const auto size = cv::getTextSize(text, font, scale, thickness, &baseline);
    const auto x    = std::max(canvas.details().get_cols() - size.width - margin, 0);
    const auto y    = std::max(canvas.details().get_rows() - baseline - margin, size.height);
    const auto org  = cv::Point2i { x, y };

    cv::putText(opencv_mat, text, org + cv::Point2i { 1, 1 }, font, scale, black, thickness + 2,
        cv::LINE_AA);
    cv::putText(opencv_mat, text, org, font, scale, white, thickness, cv::LINE_AA);
}

}
