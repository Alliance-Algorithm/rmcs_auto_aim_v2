#include "lightbar.hpp"

#include "utility/math/angle.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <opencv2/imgproc.hpp>

namespace rmcs {

auto LightbarFinder::solve() -> bool {
    if (input.roi.empty()) return false;
    if (input.color == CampColor::UNKNOWN) return false;

    auto channel = cv::Mat { };
    cv::extractChannel(input.roi, channel, input.color == CampColor::BLUE ? 0 : 2);

    auto mask = cv::Mat { };
    cv::threshold(channel, mask, 180.0, 255.0, cv::THRESH_BINARY);

    auto lines = std::vector<cv::Vec4i> { };
    cv::HoughLinesP(mask, lines, 1.0, std::numbers::pi / 180.0, 12, 12.0, 4.0);
    if (lines.empty()) return false;

    const auto predicted_dx = static_cast<double>(input.predicted_point2.x - input.predicted_point1.x);
    const auto predicted_dy = static_cast<double>(input.predicted_point2.y - input.predicted_point1.y);
    const auto predicted_angle = std::atan2(predicted_dy, predicted_dx);
    const auto predicted_midpoint = cv::Point2d {
        0.5 * static_cast<double>(input.predicted_point1.x + input.predicted_point2.x),
        0.5 * static_cast<double>(input.predicted_point1.y + input.predicted_point2.y),
    };

    auto best_line   = cv::Vec4i { };
    auto best_score  = std::numeric_limits<double>::infinity();

    for (const auto& line : lines) {
        const auto dx          = static_cast<double>(line[2] - line[0]);
        const auto dy          = static_cast<double>(line[3] - line[1]);
        const auto line_angle  = std::atan2(dy, dx);
        const auto angle_error = std::abs(util::normalize_angle(line_angle - predicted_angle));
        if (angle_error > util::deg2rad(20.0)) continue;

        const auto length = std::hypot(dx, dy);
        auto const midpoint = cv::Point2d {
            0.5 * static_cast<double>(line[0] + line[2]),
            0.5 * static_cast<double>(line[1] + line[3]),
        };
        const auto midpoint_error = std::hypot(
            midpoint.x - predicted_midpoint.x, midpoint.y - predicted_midpoint.y);
        const auto score = angle_error * 180.0 / std::numbers::pi + 0.15 * midpoint_error - 0.05 * length;
        if (score >= best_score) continue;

        best_score = score;
        best_line  = line;
    }
    if (!std::isfinite(best_score)) return false;

    auto point_a = cv::Point2i { best_line[0], best_line[1] };
    auto point_b = cv::Point2i { best_line[2], best_line[3] };

    const auto distance_a1 =
        std::hypot(point_a.x - input.predicted_point1.x, point_a.y - input.predicted_point1.y);
    const auto distance_b1 =
        std::hypot(point_b.x - input.predicted_point1.x, point_b.y - input.predicted_point1.y);

    if (distance_a1 <= distance_b1) {
        result.point1 = point_a;
        result.point2 = point_b;
    } else {
        result.point1 = point_b;
        result.point2 = point_a;
    }

    return true;
}

}
