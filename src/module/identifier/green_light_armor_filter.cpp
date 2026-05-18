#include "green_light_armor_filter.hpp"

#include "module/identifier/green_light_detection.hpp"
#include "utility/image/image.details.hpp"

#include <cstddef>
#include <ranges>
#include <span>
#include <vector>

#include <opencv2/imgproc.hpp>

using namespace rmcs::identifier;

struct GreenLightArmorFilter::Impl {
    GreenLightDetection green_light_detection;

    auto initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
        return green_light_detection.initialize(yaml);
    }

    auto filter(const Image& image, std::span<const Armor2D> armors) noexcept -> std::vector<bool> {
        auto keep_mask = std::vector<bool>(armors.size(), true);
        if (armors.empty()) return keep_mask;

        auto detect_roi = compute_detect_roi(image, armors);
        if (!detect_roi.has_value()) return keep_mask;

        auto green_light_rect = green_light_detection.sync_detect(image, *detect_roi);
        if (!green_light_rect.has_value()) return keep_mask;

        const auto threshold_y = green_light_rect->y + green_light_rect->height;
        for (std::size_t index = 0; index < armors.size(); ++index) {
            keep_mask[index] = armors[index].center.y >= 1.0 * threshold_y;
        }

        return keep_mask;
    }

    auto green_light() const noexcept -> std::optional<cv::Rect2i> {
        return green_light_detection.green_light();
    }

private:
    static auto compute_detect_roi(const Image& image, std::span<const Armor2D> armors)
        -> std::optional<cv::Rect2i> {
        constexpr auto kExpandPixels = 300;

        if (armors.empty()) return std::nullopt;

        const auto& mat = image.details().mat;
        if (mat.empty()) return std::nullopt;

        auto union_rect = cv::boundingRect(std::vector<cv::Point2f> {
            armors.front().tl,
            armors.front().tr,
            armors.front().br,
            armors.front().bl,
        });

        for (const auto& armor : armors | std::views::drop(1)) {
            const auto armor_rect = cv::boundingRect(
                std::vector<cv::Point2f> { armor.tl, armor.tr, armor.br, armor.bl });
            union_rect |= armor_rect;
        }

        auto roi = cv::Rect2i {
            union_rect.x - kExpandPixels,
            union_rect.y - kExpandPixels,
            union_rect.width + kExpandPixels * 2,
            union_rect.height + kExpandPixels * 2,
        };

        roi &= cv::Rect2i { 0, 0, mat.cols, mat.rows };
        if ((roi.width <= 0) || (roi.height <= 0)) {
            return std::nullopt;
        }

        return roi;
    }
};

auto GreenLightArmorFilter::initialize(const YAML::Node& yaml) noexcept
    -> std::expected<void, std::string> {
    return pimpl->initialize(yaml);
}

auto GreenLightArmorFilter::filter(const Image& image, std::span<const Armor2D> armors) noexcept
    -> std::vector<bool> {
    return pimpl->filter(image, armors);
}

auto GreenLightArmorFilter::green_light() const noexcept -> std::optional<cv::Rect2i> {
    return pimpl->green_light();
}

GreenLightArmorFilter::GreenLightArmorFilter() noexcept
    : pimpl { std::make_unique<Impl>() } { }

GreenLightArmorFilter::~GreenLightArmorFilter() noexcept = default;
