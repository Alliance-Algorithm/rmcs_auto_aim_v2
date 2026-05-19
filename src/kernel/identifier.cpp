#include "identifier.hpp"
#include "module/identifier/armor_detection.hpp"
#include "module/identifier/green_light_armor_filter.hpp"
#include "utility/robot/armor.hpp"

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
        auto armor_result = armor_detection.initialize(yaml);
        if (!armor_result.has_value()) return std::unexpected { armor_result.error() };
        auto filter_result = green_light_armor_filter.initialize(yaml["outpost_green_light_"
                                                                      "filter"]);
        if (!filter_result.has_value()) return std::unexpected { filter_result.error() };

        return {};
    }

    auto identify(const Image& src) noexcept -> std::optional<Identifier::Result> {
        auto detected_armors = armor_detection.sync_detect(src);
        if (!detected_armors.has_value()) return std::nullopt;

        auto outpost_armors = std::vector<Armor2D> {};
        outpost_armors.reserve(detected_armors->size());
        for (const auto& armor : *detected_armors) {
            if (armor.genre == DeviceId::OUTPOST) outpost_armors.push_back(armor);
        }

        const auto filter_result = green_light_armor_filter.filter(src, outpost_armors);

        auto filtered = Armor2Ds {};
        filtered.reserve(detected_armors->size());

        auto keep_it = filter_result.keep_mask.begin();
        for (const auto& armor : *detected_armors) {
            if (armor.genre != DeviceId::OUTPOST) {
                filtered.push_back(armor);
                continue;
            }

            if (*keep_it) filtered.push_back(armor);
            ++keep_it;
        }

        return Identifier::Result {
            .armors      = std::move(filtered),
            .green_light = filter_result.green_light,
        };
    }
};

auto Identifier::initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
    return pimpl->initialize(yaml);
}

auto Identifier::sync_identify(const Image& src) noexcept -> std::optional<Result> {
    return pimpl->identify(src);
}

Identifier::Identifier() noexcept
    : pimpl { std::make_unique<Impl>() } { }

Identifier::~Identifier() noexcept = default;
