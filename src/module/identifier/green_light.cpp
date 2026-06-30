#include "green_light.hpp"

#include "utility/serializable.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <span>

#include <opencv2/imgproc.hpp>

using namespace rmcs::identifier;

struct GreenLightFinder::Impl {
    struct Config : util::Serializable {
        int green_threshold;

        double min_area;
        double min_circularity;
        double max_aspect_ratio;

        constexpr static std::tuple metas {
            &Config::green_threshold,
            "green_threshold",
            &Config::min_area,
            "min_area",
            &Config::min_circularity,
            "min_circularity",
            &Config::max_aspect_ratio,
            "max_aspect_ratio",
        };
    } config;

    auto initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
        auto result = config.serialize(yaml);
        if (!result.has_value()) {
            return std::unexpected { result.error() };
        }

        if ((config.green_threshold < 0) || (config.green_threshold > 255)) {
            return std::unexpected { "green_threshold must satisfy 0 <= green_threshold <= 255" };
        }

        if (!(config.min_area > 0.0)) {
            return std::unexpected { "min_area must be > 0" };
        }

        if ((config.min_circularity <= 0.0) || (config.min_circularity > 1.0)) {
            return std::unexpected { "min_circularity must satisfy 0 < min_circularity <= 1" };
        }

        if (!(config.max_aspect_ratio >= 1.0)) {
            return std::unexpected { "max_aspect_ratio must be >= 1" };
        }

        return { };
    }

    auto locate(const cv::Mat& mat, std::span<const Armor2d> armors) const noexcept -> Result {
        constexpr auto kExpandScale = 4.0;
        constexpr auto kGreenMargin = 30.0;

        if (armors.empty()) return { };
        if (mat.empty()) return { };

        auto roi = cv::Rect2i { };
        { // 找出包围所有 Armors 并带有一定拓展的搜索区域，用于检索绿灯
            auto points      = std::vector<cv::Point2f> { };
            auto max_armor_w = 0.f;
            auto max_armor_h = 0.f;
            for (const auto& armor : armors) {
                const auto corners = armor.points();
                points.insert(points.end(), corners.begin(), corners.end());

                const auto rect = cv::boundingRect(corners);
                max_armor_w     = std::max(max_armor_w, static_cast<float>(rect.width));
                max_armor_h     = std::max(max_armor_h, static_cast<float>(rect.height));
            }

            const auto [min_x, max_x] = std::minmax_element(points.begin(), points.end(),
                [](const auto& a, const auto& b) { return a.x < b.x; });
            const auto [min_y, max_y] = std::minmax_element(points.begin(), points.end(),
                [](const auto& a, const auto& b) { return a.y < b.y; });

            const auto roi_left   = min_x->x - max_armor_w - 20;
            const auto roi_right  = max_x->x + max_armor_w + 20;
            const auto roi_top    = min_y->y - max_armor_h - 20;
            const auto roi_bottom = max_y->y + max_armor_h * kExpandScale;

            roi = cv::Rect2i {
                static_cast<int>(std::floor(roi_left)),
                static_cast<int>(std::floor(roi_top)),
                static_cast<int>(std::ceil(roi_right) - std::floor(roi_left)),
                static_cast<int>(std::ceil(roi_bottom) - std::floor(roi_top)),
            };

            roi &= cv::Rect2i { 0, 0, mat.cols, mat.rows };
            if ((roi.width <= 0) || (roi.height <= 0)) {
                return { };
            }
        }

        auto green_light = std::optional<cv::Rect2i> { };
        {
            const auto source = mat(roi);

            auto bchannel = cv::Mat { };
            auto gchannel = cv::Mat { };
            auto rchannel = cv::Mat { };
            cv::extractChannel(source, bchannel, 0);
            cv::extractChannel(source, gchannel, 1);
            cv::extractChannel(source, rchannel, 2);

            auto mask_g_min  = cv::Mat { };
            auto mask_g_gt_r = cv::Mat { };
            auto mask_g_gt_b = cv::Mat { };

            cv::threshold(gchannel, mask_g_min, static_cast<double>(config.green_threshold), 255.0,
                cv::THRESH_BINARY);
            cv::subtract(gchannel, rchannel, mask_g_gt_r);
            cv::subtract(gchannel, bchannel, mask_g_gt_b);
            cv::threshold(mask_g_gt_r, mask_g_gt_r, kGreenMargin, 255.0, cv::THRESH_BINARY);
            cv::threshold(mask_g_gt_b, mask_g_gt_b, kGreenMargin, 255.0, cv::THRESH_BINARY);

            auto mask = cv::Mat { };
            cv::bitwise_and(mask_g_min, mask_g_gt_r, mask);
            cv::bitwise_and(mask, mask_g_gt_b, mask);

            auto kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size { 5, 5 });
            cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

            auto contours = std::vector<std::vector<cv::Point>> { };
            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            auto best_area = 0.0;

            for (const auto& contour : contours) {
                const auto area = cv::contourArea(contour);
                if (area < config.min_area) {
                    continue;
                }

                const auto perimeter = cv::arcLength(contour, true);
                if (!(perimeter > 0.0)) {
                    continue;
                }

                const auto circularity = 4.0 * std::numbers::pi * area / (perimeter * perimeter);
                if (circularity < config.min_circularity) {
                    continue;
                }

                const auto rect = cv::boundingRect(contour);
                if ((rect.width <= 0) || (rect.height <= 0)) {
                    continue;
                }

                const auto aspect_ratio =
                    1. * std::max(rect.width, rect.height) / std::min(rect.width, rect.height);
                if (aspect_ratio > config.max_aspect_ratio) {
                    continue;
                }

                if (area > best_area) {
                    best_area   = area;
                    green_light = rect;
                    green_light->x += roi.x;
                    green_light->y += roi.y;
                }
            }
        }
        return { roi, green_light };
    }
};

auto GreenLightFinder::initialize(const YAML::Node& yaml) noexcept
    -> std::expected<void, std::string> {
    return pimpl->initialize(yaml);
}

auto GreenLightFinder::locate(const cv::Mat& image, std::span<const Armor2d> armors) noexcept
    -> Result {
    return pimpl->locate(image, armors);
}

GreenLightFinder::GreenLightFinder() noexcept
    : pimpl { std::make_unique<Impl>() } { }

GreenLightFinder::~GreenLightFinder() noexcept = default;
