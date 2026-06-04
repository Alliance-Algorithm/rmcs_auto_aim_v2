#include "green_light_detection.hpp"

#include "utility/image/image.details.hpp"
#include "utility/serializable.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <optional>

#include <opencv2/imgproc.hpp>

using namespace rmcs::identifier;

struct GreenLightDetection::Impl {
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

    auto sync_detect(const Image& image, const cv::Rect2i& roi) const noexcept
        -> std::optional<cv::Rect2i> {
        auto green_light = std::optional<cv::Rect2i> { };

        const auto& mat = image.details().mat;
        if (mat.empty()) {
            return std::nullopt;
        }

        if ((roi.width <= 0) || (roi.height <= 0)) {
            return std::nullopt;
        }

        const auto image_rect  = cv::Rect2i { 0, 0, mat.cols, mat.rows };
        const auto clipped_roi = roi & image_rect;
        if ((clipped_roi.width != roi.width) || (clipped_roi.height != roi.height)) {
            return std::nullopt;
        }

        const auto roi_mat = mat(roi);

        auto bchannel = cv::Mat { };
        auto gchannel = cv::Mat { };
        auto rchannel = cv::Mat { };
        cv::extractChannel(roi_mat, bchannel, 0);
        cv::extractChannel(roi_mat, gchannel, 1);
        cv::extractChannel(roi_mat, rchannel, 2);

        constexpr auto kGreenMargin = 30.0;

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

            const auto circularity =
                4.0 * std::numbers::pi_v<double> * area / (perimeter * perimeter);
            if (circularity < config.min_circularity) {
                continue;
            }

            const auto rect = cv::boundingRect(contour);
            if ((rect.width <= 0) || (rect.height <= 0)) {
                continue;
            }

            const auto aspect_ratio = static_cast<double>(std::max(rect.width, rect.height))
                / std::min(rect.width, rect.height);
            if (aspect_ratio > config.max_aspect_ratio) {
                continue;
            }

            if (!green_light.has_value() || (area > best_area)) {
                best_area   = area;
                green_light = rect;
                green_light->x += roi.x;
                green_light->y += roi.y;
            }
        }

        return green_light;
    }
};

auto GreenLightDetection::initialize(const YAML::Node& yaml) noexcept
    -> std::expected<void, std::string> {
    return pimpl->initialize(yaml);
}

auto GreenLightDetection::sync_detect(const Image& image, const cv::Rect2i& roi) noexcept
    -> std::optional<cv::Rect2i> {
    return pimpl->sync_detect(image, roi);
}

GreenLightDetection::GreenLightDetection() noexcept
    : pimpl { std::make_unique<Impl>() } { }

GreenLightDetection::~GreenLightDetection() noexcept = default;
