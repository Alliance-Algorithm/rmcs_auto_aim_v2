#include "identifier.hpp"

#include "module/identifier/armor_detection.hpp"
#include "module/identifier/green_light_locator.hpp"

#include "utility/image/green_light.hpp"
#include "utility/robot/armor.hpp"

#include <optional>
#include <span>
#include <vector>

using namespace rmcs;
using namespace rmcs::kernel;
using namespace rmcs::identifier;

struct Identifier::Impl {
    ArmorDetection armor_detection;
    GreenLightLocator green_light_locator;
    std::optional<cv::Rect2i> outpost_green_light;
    std::optional<cv::Rect2i> base_green_light;
    std::optional<cv::Rect2i> outpost_green_light_roi;
    std::optional<cv::Rect2i> base_green_light_roi;

    auto initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
        auto armor_result = armor_detection.initialize(yaml);
        if (!armor_result.has_value()) return std::unexpected { armor_result.error() };
        auto locator_result = green_light_locator.initialize(yaml["green_light_filter"]);
        if (!locator_result.has_value()) return std::unexpected { locator_result.error() };

        return { };
    }

    auto identify(const Image& src) noexcept -> std::optional<Identifier::Result> {
        auto detected_armors = armor_detection.sync_detect(src);
        if (!detected_armors.has_value()) return std::nullopt;

        auto outpost_armors = std::vector<Armor2D> { };
        auto base_armors    = std::vector<Armor2D> { };
        outpost_armors.reserve(detected_armors->size());
        base_armors.reserve(detected_armors->size());
        for (const auto& armor : *detected_armors) {
            if (armor.genre == DeviceId::OUTPOST) outpost_armors.push_back(armor);
            if (armor.genre == DeviceId::BASE) base_armors.push_back(armor);
        }

        const auto outpost_locator_result = green_light_locator.locate(src, outpost_armors);
        const auto base_locator_result    = green_light_locator.locate(src, base_armors);

        outpost_green_light     = outpost_locator_result.green_light;
        base_green_light        = base_locator_result.green_light;
        outpost_green_light_roi = outpost_locator_result.detect_roi;
        base_green_light_roi    = base_locator_result.detect_roi;

        auto filtered = Armor2Ds { };
        filtered.reserve(detected_armors->size());

        if (!outpost_locator_result.green_light.has_value()
            && !base_locator_result.green_light.has_value()) {
            filtered = *detected_armors;
        } else {
            const auto outpost_threshold_y = outpost_locator_result.green_light.has_value()
                ? std::optional {
                      outpost_locator_result.green_light->y
                      + outpost_locator_result.green_light->height,
                  }
                : std::nullopt;
            const auto base_threshold_y = base_locator_result.green_light.has_value()
                ? std::optional {
                      base_locator_result.green_light->y + base_locator_result.green_light->height,
                  }
                : std::nullopt;

            for (const auto& armor : *detected_armors) {
                const auto threshold_y = armor.genre == DeviceId::OUTPOST ? outpost_threshold_y
                    : armor.genre == DeviceId::BASE                       ? base_threshold_y
                                                                          : std::nullopt;
                // 过滤掉绿灯之上的对应装甲板（图像坐标系 y 向下为正）
                if (threshold_y.has_value() && (armor.center.y < 1.0 * *threshold_y)) continue;

                filtered.push_back(armor);
            }
        }

        return Identifier::Result {
            .armors = std::move(filtered),
        };
    }

    auto draw_green_light(Image& image) noexcept -> void {
        if (outpost_green_light.has_value()) {
            util::draw_green_light(image, *outpost_green_light);
        }
        if (base_green_light.has_value()) {
            util::draw_green_light(image, *base_green_light);
        }
    }

    auto draw_green_light_roi(Image& image) noexcept -> void {
        if (outpost_green_light_roi.has_value()) {
            util::draw_green_light_roi(image, *outpost_green_light_roi);
        }
        if (base_green_light_roi.has_value()) {
            util::draw_green_light_roi(image, *base_green_light_roi);
        }
    }
};

auto Identifier::initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
    return pimpl->initialize(yaml);
}

auto Identifier::sync_identify(const Image& src) noexcept -> std::optional<Result> {
    return pimpl->identify(src);
}

auto Identifier::draw_green_light(Image& image) noexcept -> void {
    return pimpl->draw_green_light(image);
}

auto Identifier::draw_green_light_roi(Image& image) noexcept -> void {
    return pimpl->draw_green_light_roi(image);
}

Identifier::Identifier() noexcept
    : pimpl { std::make_unique<Impl>() } { }

Identifier::~Identifier() noexcept = default;
