#include "lightbar.hpp"

#include "utility/math/angle.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace rmcs {

auto LightbarFinder::solve() -> bool {
    if (input.source.empty()) return false;
    if (input.color == CampColor::UNKNOWN) return false;

    auto channel = cv::Mat { };
    cv::extractChannel(input.source, channel, input.color == CampColor::BLUE ? 0 : 2);

    auto mask = cv::Mat { };
    cv::threshold(channel, mask, 180.0, 255.0, cv::THRESH_BINARY);

    auto contours = std::vector<std::vector<cv::Point>> { };
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    if (contours.empty()) return false;

    const auto predicted_dx =
        static_cast<double>(input.predicted_upper.x - input.predicted_lower.x);
    const auto predicted_dy =
        static_cast<double>(input.predicted_upper.y - input.predicted_lower.y);
    const auto predicted_angle    = std::atan2(predicted_dy, predicted_dx);
    const auto predicted_length   = std::hypot(predicted_dx, predicted_dy);
    const auto predicted_midpoint = cv::Point2d {
        0.5 * static_cast<double>(input.predicted_upper.x + input.predicted_lower.x),
        0.5 * static_cast<double>(input.predicted_upper.y + input.predicted_lower.y),
    };

    auto best_top    = cv::Point2f { };
    auto best_bottom = cv::Point2f { };
    auto best_score  = std::numeric_limits<double>::infinity();

    for (const auto& contour : contours) {
        if (contour.size() < 5) continue;
        if (cv::contourArea(contour) < 3.0) continue;

        const auto box    = cv::minAreaRect(contour);
        const auto width  = static_cast<double>(box.size.width);
        const auto height = static_cast<double>(box.size.height);
        if (width <= 1.0 || height <= 1.0) continue;

        const auto long_edge  = std::max(width, height);
        const auto short_edge = std::min(width, height);
        if (short_edge <= 1.0) continue;
        if (long_edge / short_edge < 1.8) continue;

        cv::Point2f corners[4] { };
        box.points(corners);

        const auto edge01 = cv::norm(corners[0] - corners[1]);
        const auto edge12 = cv::norm(corners[1] - corners[2]);

        auto point_a = cv::Point2f { };
        auto point_b = cv::Point2f { };
        if (edge01 >= edge12) {
            point_a = 0.5f * (corners[1] + corners[2]);
            point_b = 0.5f * (corners[3] + corners[0]);
        } else {
            point_a = 0.5f * (corners[0] + corners[1]);
            point_b = 0.5f * (corners[2] + corners[3]);
        }

        const auto dx          = static_cast<double>(point_b.x - point_a.x);
        const auto dy          = static_cast<double>(point_b.y - point_a.y);
        const auto length      = std::hypot(dx, dy);
        const auto line_angle  = std::atan2(dy, dx);
        const auto angle_diff  = util::normalize_angle(line_angle - predicted_angle);
        const auto angle_error = std::min(
            std::abs(angle_diff), std::abs(util::normalize_angle(angle_diff + std::numbers::pi)));
        if (angle_error > util::deg2rad(30.0)) continue;

        auto midpoint = cv::Point2d {
            0.5 * static_cast<double>(point_a.x + point_b.x),
            0.5 * static_cast<double>(point_a.y + point_b.y),
        };
        const auto midpoint_error =
            std::hypot(midpoint.x - predicted_midpoint.x, midpoint.y - predicted_midpoint.y);
        const auto length_error = std::abs(length - predicted_length);

        // 保留预测约束，只把端点提取替换成轮廓矩形法。
        const auto score = angle_error * 180.0 / std::numbers::pi + 0.12 * midpoint_error
            + 0.04 * length_error - 0.08 * length;
        if (score >= best_score) continue;

        best_score = score;
        if (point_a.y <= point_b.y) {
            best_top    = point_a;
            best_bottom = point_b;
        } else {
            best_top    = point_b;
            best_bottom = point_a;
        }
    }
    if (!std::isfinite(best_score)) return false;

    auto point_a = cv::Point2i { static_cast<int>(std::lround(best_top.x)),
        static_cast<int>(std::lround(best_top.y)) };
    auto point_b = cv::Point2i { static_cast<int>(std::lround(best_bottom.x)),
        static_cast<int>(std::lround(best_bottom.y)) };

    const auto distance_a1 =
        std::hypot(point_a.x - input.predicted_upper.x, point_a.y - input.predicted_upper.y);
    const auto distance_b1 =
        std::hypot(point_b.x - input.predicted_upper.x, point_b.y - input.predicted_upper.y);

    if (distance_a1 <= distance_b1) {
        result.upper = point_a;
        result.lower = point_b;
    } else {
        result.upper = point_b;
        result.lower = point_a;
    }

    return true;
}

}
