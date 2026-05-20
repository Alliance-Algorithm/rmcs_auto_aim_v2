#include "green_light_locator.hpp"

#include "module/identifier/green_light_detection.hpp"
#include "utility/image/image.details.hpp"

#include <ranges>
#include <span>
#include <vector>

#include <opencv2/imgproc.hpp>

using namespace rmcs::identifier;

struct GreenLightLocator::Impl {
    GreenLightDetection green_light_detection;

    auto initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
        return green_light_detection.initialize(yaml);
    }

    auto locate(const Image& image, std::span<const Armor2D> armors) noexcept
        -> GreenLightLocator::Result {
        auto result = GreenLightLocator::Result {
            .green_light = std::nullopt,
        };
        if (armors.empty()) return result;

        auto detect_roi = compute_detect_roi(image, armors);
        if (!detect_roi.has_value()) return result;

        result.green_light = green_light_detection.sync_detect(image, *detect_roi);
        return result;
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

auto GreenLightLocator::initialize(const YAML::Node& yaml) noexcept
    -> std::expected<void, std::string> {
    return pimpl->initialize(yaml);
}

auto GreenLightLocator::locate(const Image& image, std::span<const Armor2D> armors) noexcept
    -> Result {
    return pimpl->locate(image, armors);
}

GreenLightLocator::GreenLightLocator() noexcept
    : pimpl { std::make_unique<Impl>() } { }

GreenLightLocator::~GreenLightLocator() noexcept = default;
