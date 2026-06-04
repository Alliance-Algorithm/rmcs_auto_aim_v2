#include "green_light_locator.hpp"

#include "module/identifier/green_light_detection.hpp"
#include "utility/image/image.details.hpp"

#include <algorithm>
#include <cmath>
#include <span>

using namespace rmcs::identifier;

struct GreenLightLocator::Impl {
    GreenLightDetection green_light_detection;

    auto initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
        return green_light_detection.initialize(yaml);
    }

    auto locate(const Image& image, std::span<const Armor2D> armors) noexcept
        -> GreenLightLocator::Result {
        auto result = GreenLightLocator::Result {
            .detect_roi  = std::nullopt,
            .green_light = std::nullopt,
        };
        if (armors.empty()) return result;

        auto detect_roi = compute_detect_roi(image, armors);
        if (!detect_roi.has_value()) return result;

        result.detect_roi  = detect_roi;
        result.green_light = green_light_detection.sync_detect(image, *detect_roi);
        return result;
    }

private:
    static auto compute_detect_roi(const Image& image, std::span<const Armor2D> armors)
        -> std::optional<cv::Rect2i> {
        constexpr auto kExpandScale = 4.0;

        if (armors.empty()) return std::nullopt;

        const auto& mat = image.details().mat;
        if (mat.empty()) return std::nullopt;

        auto min_x            = armors.front().tl.x;
        auto max_x            = armors.front().tl.x;
        auto min_y            = armors.front().tl.y;
        auto max_y            = armors.front().tl.y;
        auto max_armor_width  = 0.0F;
        auto max_armor_height = 0.0F;

        for (const auto& armor : armors) {
            auto armor_min_x = armor.tl.x;
            auto armor_max_x = armor.tl.x;
            auto armor_min_y = armor.tl.y;
            auto armor_max_y = armor.tl.y;

            for (const auto& corner : armor.corners()) {
                min_x = std::min(min_x, corner.x);
                max_x = std::max(max_x, corner.x);
                min_y = std::min(min_y, corner.y);
                max_y = std::max(max_y, corner.y);

                armor_min_x = std::min(armor_min_x, corner.x);
                armor_max_x = std::max(armor_max_x, corner.x);
                armor_min_y = std::min(armor_min_y, corner.y);
                armor_max_y = std::max(armor_max_y, corner.y);
            }

            max_armor_width  = std::max(max_armor_width, armor_max_x - armor_min_x);
            max_armor_height = std::max(max_armor_height, armor_max_y - armor_min_y);
        }

        const auto roi_left   = min_x - max_armor_width * kExpandScale;
        const auto roi_right  = max_x + max_armor_width * kExpandScale;
        const auto roi_top    = min_y - max_armor_height * kExpandScale;
        const auto roi_bottom = max_y + max_armor_height * kExpandScale;

        auto roi = cv::Rect2i {
            static_cast<int>(std::floor(roi_left)),
            static_cast<int>(std::floor(roi_top)),
            static_cast<int>(std::ceil(roi_right) - std::floor(roi_left)),
            static_cast<int>(std::ceil(roi_bottom) - std::floor(roi_top)),
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
