#include "rune.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <ranges>
#include <string_view>
#include <vector>

namespace rmcs {

namespace details {
    constexpr auto kBullseyeRadius = 0.15;
    constexpr auto kRuneIconRadius = 0.05;

    auto resample_contour(const std::vector<cv::Point>& contour, size_t n)
        -> std::vector<cv::Point2f> {
        if (contour.size() < 2 || n < 2) return { };

        auto seg_len = std::vector<double> { };
        seg_len.reserve(contour.size());
        auto total = 0.0;
        for (size_t i = 1; i < contour.size(); ++i) {
            const auto dx = static_cast<double>(contour[i].x - contour[i - 1].x);
            const auto dy = static_cast<double>(contour[i].y - contour[i - 1].y);
            seg_len.push_back(std::hypot(dx, dy));
            total += seg_len.back();
        }
        {
            const auto dx = static_cast<double>(contour[0].x - contour.back().x);
            const auto dy = static_cast<double>(contour[0].y - contour.back().y);
            seg_len.push_back(std::hypot(dx, dy));
            total += seg_len.back();
        }

        auto sampled = std::vector<cv::Point2f> { };
        sampled.reserve(n);
        const auto step  = total / static_cast<double>(n);
        const auto seg_n = seg_len.size();
        auto acc         = 0.0;
        auto seg_idx     = size_t { 0 };

        for (size_t i = 0; i < n; ++i) {
            const auto target = static_cast<double>(i) * step;
            while (seg_idx < seg_n && acc + seg_len[seg_idx] < target) {
                acc += seg_len[seg_idx];
                ++seg_idx;
            }
            if (seg_idx >= seg_n) seg_idx = seg_n - 1;
            const auto alpha = seg_len[seg_idx] > 0.0 ? (target - acc) / seg_len[seg_idx] : 0.0;
            const auto& p0   = contour[seg_idx % contour.size()];
            const auto& p1   = contour[(seg_idx + 1) % contour.size()];
            sampled.emplace_back(static_cast<float>(p0.x + alpha * (p1.x - p0.x)),
                static_cast<float>(p0.y + alpha * (p1.y - p0.y)));
        }

        return sampled;
    }

    using ShapeContextHist = std::array<float, 60>;

    constexpr auto kShapeContextBinsR     = 5;
    constexpr auto kShapeContextBinsTheta = 12;
    constexpr auto kShapeContextN         = 100;

    struct LogDistRange {
        double min;
        double max;
    };

    static const auto kTemplateLogDistRange = [] {
        constexpr auto kTemplate = std::array {
            std::string_view { " ###################    " },
            std::string_view { " ####################   " },
            std::string_view { "  ####################  " },
            std::string_view { "   #################### " },
            std::string_view { "   #################### " },
            std::string_view { "    #######       ######" },
            std::string_view { "     #######       #####" },
            std::string_view { "     #######       #####" },
            std::string_view { "      #######      #####" },
            std::string_view { "   #################### " },
            std::string_view { "   #################### " },
            std::string_view { "   ###################  " },
            std::string_view { "  ###################   " },
            std::string_view { "  #################     " },
            std::string_view { "  ######    #######     " },
            std::string_view { "  ######     ######     " },
            std::string_view { " ######      #######    " },
            std::string_view { " ######       #######   " },
            std::string_view { " ######        #######  " },
            std::string_view { " ######    ############ " },
            std::string_view { "######      ########### " },
            std::string_view { "######        #####     " },
            std::string_view { "                ####    " },
            std::string_view { "                  ###   " },
            std::string_view { "                    ##  " },
            std::string_view { "                      # " },
        };

        const auto rows = static_cast<int>(kTemplate.size());
        const auto cols = static_cast<int>(kTemplate[0].size());

        auto mat = cv::Mat(rows, cols, CV_8UC1, cv::Scalar { 0 });
        for (const auto [r, row] : kTemplate | std::views::enumerate) {
            for (const auto [c, ch] : row | std::views::enumerate) {
                if (ch == '#') mat.at<uint8_t>(static_cast<int>(r), static_cast<int>(c)) = 255;
            }
        }

        auto contours = std::vector<std::vector<cv::Point>> { };
        cv::findContours(mat, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        if (contours.empty()) return LogDistRange { 0.0, 1.0 };

        const auto& largest = *std::ranges::max_element(
            contours, { }, [](const auto& c) { return cv::contourArea(c); });

        const auto sampled  = resample_contour(largest, kShapeContextN);
        constexpr auto kEps = 1e-6;

        auto all_dists = std::vector<double> { };
        all_dists.reserve(sampled.size() * sampled.size() / 2);
        for (size_t i = 0; i < sampled.size(); ++i) {
            for (size_t j = i + 1; j < sampled.size(); ++j) {
                const auto dx = static_cast<double>(sampled[j].x - sampled[i].x);
                const auto dy = static_cast<double>(sampled[j].y - sampled[i].y);
                all_dists.push_back(std::log(std::sqrt(dx * dx + dy * dy) + kEps));
            }
        }

        const auto [r_min, r_max] = std::ranges::minmax(all_dists);
        auto range                = r_max - r_min;
        if (range < 1.0) range = 1.0;
        return LogDistRange { r_min - 0.2 * range, r_max + 0.2 * range };
    }();

    auto compute_shape_context(const std::vector<cv::Point>& contour, size_t n = kShapeContextN)
        -> ShapeContextHist {
        const auto sampled = resample_contour(contour, n);
        if (sampled.size() != n) return { };

        auto hist            = ShapeContextHist { };
        constexpr auto n_r   = static_cast<double>(kShapeContextBinsR);
        constexpr auto n_t   = static_cast<double>(kShapeContextBinsTheta);
        constexpr auto pi    = std::numbers::pi;
        constexpr auto twopi = 2.0 * pi;
        constexpr auto kEps  = 1e-6;

        const auto global_r_min = kTemplateLogDistRange.min;
        const auto global_r_max = kTemplateLogDistRange.max;
        auto global_r_range     = global_r_max - global_r_min;
        if (global_r_range < 1e-3) global_r_range = 1.0;

        for (size_t i = 0; i < n; ++i) {
            auto log_dists = std::vector<double> { };
            auto angles    = std::vector<double> { };
            log_dists.reserve(n - 1);
            angles.reserve(n - 1);
            for (size_t j = 0; j < n; ++j) {
                if (i == j) continue;
                const auto dx = static_cast<double>(sampled[j].x - sampled[i].x);
                const auto dy = static_cast<double>(sampled[j].y - sampled[i].y);
                log_dists.push_back(std::log(std::sqrt(dx * dx + dy * dy) + kEps));
                angles.push_back(std::atan2(dy, dx));
            }

            for (size_t k = 0; k < log_dists.size(); ++k) {
                const auto r_bin = std::clamp(
                    static_cast<int>((log_dists[k] - global_r_min) / global_r_range * n_r), 0,
                    kShapeContextBinsR - 1);
                const auto theta_bin = std::clamp(static_cast<int>((angles[k] + pi) / twopi * n_t),
                    0, kShapeContextBinsTheta - 1);
                hist[static_cast<size_t>(r_bin) * kShapeContextBinsTheta
                    + static_cast<size_t>(theta_bin)] += 1.0f;
            }
        }

        for (auto& h : hist)
            h /= static_cast<float>(n);
        auto sum = 0.0f;
        for (const auto& h : hist)
            sum += h;
        if (sum > 0.0f)
            for (auto& h : hist)
                h /= sum;

        return hist;
    }

    static const auto kTemplateShapeContext = [] {
        constexpr auto kTemplate = std::array {
            std::string_view { " ###################    " },
            std::string_view { " ####################   " },
            std::string_view { "  ####################  " },
            std::string_view { "   #################### " },
            std::string_view { "   #################### " },
            std::string_view { "    #######       ######" },
            std::string_view { "     #######       #####" },
            std::string_view { "     #######       #####" },
            std::string_view { "      #######      #####" },
            std::string_view { "   #################### " },
            std::string_view { "   #################### " },
            std::string_view { "   ###################  " },
            std::string_view { "  ###################   " },
            std::string_view { "  #################     " },
            std::string_view { "  ######    #######     " },
            std::string_view { "  ######     ######     " },
            std::string_view { " ######      #######    " },
            std::string_view { " ######       #######   " },
            std::string_view { " ######        #######  " },
            std::string_view { " ######    ############ " },
            std::string_view { "######      ########### " },
            std::string_view { "######        #####     " },
            std::string_view { "                ####    " },
            std::string_view { "                  ###   " },
            std::string_view { "                    ##  " },
            std::string_view { "                      # " },
        };

        const auto rows = static_cast<int>(kTemplate.size());
        const auto cols = static_cast<int>(kTemplate[0].size());

        auto mat = cv::Mat(rows, cols, CV_8UC1, cv::Scalar { 0 });
        for (const auto [r, row] : kTemplate | std::views::enumerate) {
            for (const auto [c, ch] : row | std::views::enumerate) {
                if (ch == '#') mat.at<uint8_t>(static_cast<int>(r), static_cast<int>(c)) = 255;
            }
        }

        auto contours = std::vector<std::vector<cv::Point>> { };
        cv::findContours(mat, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        if (contours.empty()) return ShapeContextHist { };

        const auto& largest = *std::ranges::max_element(
            contours, { }, [](const auto& c) { return cv::contourArea(c); });

        return compute_shape_context(largest, 100);
    }();

    auto shape_context_distance(const ShapeContextHist& a, const ShapeContextHist& b) -> double {
        auto dist = 0.0;
        for (size_t i = 0; i < a.size(); ++i) {
            const auto diff = static_cast<double>(a[i]) - static_cast<double>(b[i]);
            const auto sum  = static_cast<double>(a[i]) + static_cast<double>(b[i]) + 1e-10;
            dist += diff * diff / sum;
        }
        return dist;
    }

    auto find_contours(const RuneDetector::Config& config, const cv::Mat& mat,
        std::vector<std::vector<cv::Point>>& icons,
        std::vector<std::vector<cv::Point>>& bullseyes) noexcept {

        icons.clear();
        bullseyes.clear();
        if (mat.empty() || mat.type() != CV_8UC1) return;

        const auto fx = config.cam.camera_matrix[0][0];
        const auto fy = config.cam.camera_matrix[1][1];
        if (fx <= 0.0 || fy <= 0.0) return;
        const auto focal = (fx + fy) * 0.5;

        const auto perspective_rad = config.max_perspective * std::numbers::pi / 180.0;
        const auto cos_theta       = std::cos(perspective_rad);
        if (cos_theta <= 0.0) return;

        const auto radius_range = [=](double physical_radius, double min_distance,
                                      double max_distance) -> std::array<double, 2> {
            constexpr auto kMargin = 0.1;

            const auto r_min = focal * physical_radius * cos_theta / max_distance * (1.0 - kMargin);
            const auto r_max =
                focal * physical_radius / (min_distance * cos_theta) * (1.0 + kMargin);

            return { r_min, r_max };
        };
        const auto area_range = [=](double physical_radius, double min_distance,
                                    double max_distance,
                                    double fill_ratio) -> std::array<double, 2> {
            constexpr auto kMargin = 0.1;

            const auto a_min = focal * focal * physical_radius * physical_radius * cos_theta
                / (max_distance * max_distance) * (1.0 - kMargin) * std::numbers::pi * fill_ratio;
            const auto a_max = focal * focal * physical_radius * physical_radius
                / (min_distance * min_distance * cos_theta) * (1.0 + kMargin) * std::numbers::pi;

            return { a_min, a_max };
        };

        const auto bullseye_range =
            radius_range(kBullseyeRadius, config.min_distance, config.max_distance);
        const auto icon_area_range =
            area_range(kRuneIconRadius, config.min_distance, config.max_distance, 0.2);

        const auto min_circularity = 2.0 * cos_theta / (1.0 + cos_theta * cos_theta);

        auto contours  = std::vector<std::vector<cv::Point>> { };
        auto hierarchy = std::vector<cv::Vec4i> { };
        cv::findContours(mat.clone(), contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        for (const auto& contour : contours) {
            if (contour.size() < 5) continue;

            const auto area = cv::contourArea(contour);
            if (area <= 0.0) continue;

            const auto perimeter = cv::arcLength(contour, true);
            if (perimeter <= 0.0) continue;

            const auto radius = std::sqrt(area / std::numbers::pi);

            /*^^*/ if (radius >= bullseye_range[0] && radius <= bullseye_range[1]) {
                constexpr auto kEllipseAreaRatioMin = 0.7;
                constexpr auto kEllipseAreaRatioMax = 1.3;
                constexpr auto kCircularityMargin   = 0.9;

                const auto ellipse = cv::fitEllipse(contour);
                const auto major   = std::max(ellipse.size.width, ellipse.size.height);
                const auto minor   = std::min(ellipse.size.width, ellipse.size.height);
                if (major <= 0.0 || minor <= 0.0) continue;

                const auto aspect_ratio = minor / major;
                if (aspect_ratio < cos_theta) continue;

                const auto ellipse_area = std::numbers::pi * (major * 0.5) * (minor * 0.5);
                const auto area_ratio   = area / ellipse_area;

                if (area_ratio < kEllipseAreaRatioMin || area_ratio > kEllipseAreaRatioMax)
                    continue;

                const auto circularity = 4.0 * std::numbers::pi * area / (perimeter * perimeter);
                if (circularity < min_circularity * kCircularityMargin) continue;

                // @TODO: 同心圆校验

                bullseyes.push_back(contour);
            } else if (area >= icon_area_range[0] && area <= icon_area_range[1]) {
                const auto ellipse = cv::fitEllipse(contour);
                const auto major   = std::max(ellipse.size.width, ellipse.size.height);
                const auto minor   = std::min(ellipse.size.width, ellipse.size.height);
                if (major <= 0.0 || minor <= 0.0) continue;

                const auto aspect_ratio = minor / major;
                if (aspect_ratio < cos_theta) continue;

                icons.push_back(contour);
            }
        }

        auto filtered_icons = std::vector<std::vector<cv::Point>> { };
        std::ranges::copy(icons | std::views::filter([&bullseyes](const auto& icon) {
            const auto moment = cv::moments(icon);
            const auto center = cv::Point2f {
                static_cast<float>(moment.m10 / moment.m00),
                static_cast<float>(moment.m01 / moment.m00),
            };

            return !std::ranges::any_of(bullseyes, [&center](const auto& bull) {
                return cv::pointPolygonTest(bull, center, false) >= 0;
            });
        }),
            std::back_inserter(filtered_icons));

        icons = std::move(filtered_icons);
    }

}

auto RuneDetector::detect(const cv::Mat& mat) const -> Elements {
    if (mat.empty()) return { };

    auto binary = cv::Mat { };
    {
        auto channels = std::vector<cv::Mat> { };
        cv::split(mat, channels);

        auto difference    = cv::Mat { };
        auto difference_u8 = cv::Mat { };
        auto minimum_mask  = cv::Mat { };
        auto mask          = cv::Mat { };

        /*^^*/ if (config.color == CampColor::RED) {
            cv::subtract(channels[2], channels[0], difference, cv::noArray(), CV_16S);
            difference.convertTo(difference_u8, CV_8U);
            cv::threshold(difference_u8, mask, config.red_diff_threshold, 255, cv::THRESH_BINARY);

            cv::threshold(
                channels[2], minimum_mask, config.min_channel_threshold, 255, cv::THRESH_BINARY);
        } else if (config.color == CampColor::BLUE) {
            cv::subtract(channels[0], channels[2], difference, cv::noArray(), CV_16S);
            difference.convertTo(difference_u8, CV_8U);
            cv::threshold(difference_u8, mask, config.blue_diff_threshold, 255, cv::THRESH_BINARY);

            cv::threshold(
                channels[0], minimum_mask, config.min_channel_threshold, 255, cv::THRESH_BINARY);
        } else {
            return { };
        }

        cv::bitwise_and(mask, minimum_mask, binary);
    }

    auto icons     = std::vector<std::vector<cv::Point>> { };
    auto bullseyes = std::vector<std::vector<cv::Point>> { };
    details::find_contours(config, binary, icons, bullseyes);

    auto result = Elements { };

    for (const auto& icon : icons) {
        const auto sc = details::compute_shape_context(icon, 100);

        const auto score = details::shape_context_distance(details::kTemplateShapeContext, sc);
        if (score >= config.match_threshold) continue;

        const auto rc = cv::minAreaRect(icon);
        result.icons.emplace_back(Point2d { rc.center }, score);
    }

    return result;
}

} // namespace rmcs
