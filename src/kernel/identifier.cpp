#include "identifier.hpp"
#include "module/identifier/armor_detection.hpp"
#include "module/identifier/green_light_armor_filter.hpp"
#include "utility/robot/armor.hpp"

#include <algorithm>
#include <optional>
#include <span>
#include <vector>

using namespace rmcs;
using namespace rmcs::kernel;
using namespace rmcs::identifier;

struct Identifier::Impl {
    ArmorDetection armor_detection;
    GreenLightArmorFilter green_light_armor_filter;

    auto initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
        {
            auto result = armor_detection.initialize(yaml);
            if (!result.has_value()) return std::unexpected { result.error() };
        }
        {
            auto result = green_light_armor_filter.initialize(yaml["outpost_green_light_filter"]);
            if (!result.has_value()) return std::unexpected { result.error() };
        }

        return {};
    }

    auto identify(const Image& src) noexcept -> std::optional<std::vector<Armor2D>> {
        auto detected_armors = armor_detection.sync_detect(src);
        if (!detected_armors.has_value()) return std::nullopt;

        const auto has_outpost = std::ranges::any_of(*detected_armors,
            [](const Armor2D& armor) { return armor.genre == DeviceId::OUTPOST; });
        if (!has_outpost) return detected_armors;

        auto outpost_armors = std::vector<Armor2D> {};
        outpost_armors.reserve(detected_armors->size());
        for (const auto& armor : *detected_armors) {
            if (armor.genre == DeviceId::OUTPOST) outpost_armors.push_back(armor);
        }

        const auto keep_mask = green_light_armor_filter.filter(src, outpost_armors);

        auto filtered = std::vector<Armor2D> {};
        filtered.reserve(detected_armors->size());

        auto keep_it = keep_mask.begin();
        for (const auto& armor : *detected_armors) {
            if (armor.genre != DeviceId::OUTPOST) {
                filtered.push_back(armor);
                continue;
            }

            if (*keep_it) filtered.push_back(armor);
            ++keep_it;
        }

        return filtered;
    }

    auto green_light() const noexcept -> std::optional<cv::Rect2i> {
        return green_light_armor_filter.green_light();
    }
};

auto Identifier::initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
    return pimpl->initialize(yaml);
}

auto Identifier::sync_identify(const Image& src) noexcept -> std::optional<std::vector<Armor2D>> {
    return pimpl->identify(src);
}

auto Identifier::green_light() const noexcept -> std::optional<cv::Rect2i> {
    return pimpl->green_light();
}

Identifier::Identifier() noexcept
    : pimpl { std::make_unique<Impl>() } { }

Identifier::~Identifier() noexcept = default;
