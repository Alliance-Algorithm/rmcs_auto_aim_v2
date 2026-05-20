#include "identifier.hpp"
#include "module/identifier/armor_detection.hpp"
#include "module/identifier/green_light_locator.hpp"
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

    auto initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
        auto armor_result = armor_detection.initialize(yaml);
        if (!armor_result.has_value()) return std::unexpected { armor_result.error() };
        auto locator_result = green_light_locator.initialize(yaml["outpost_green_light_"
                                                                  "filter"]);
        if (!locator_result.has_value()) return std::unexpected { locator_result.error() };

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

        const auto locator_result = green_light_locator.locate(src, outpost_armors);

        auto filtered = Armor2Ds {};
        filtered.reserve(detected_armors->size());

        if (!locator_result.green_light.has_value()) {
            filtered = *detected_armors;
        } else {
            const auto threshold_y =
                locator_result.green_light->y + locator_result.green_light->height;
            for (const auto& armor : *detected_armors) {
                const auto outpost_interference = DeviceIds::kBuilding().contains(armor.genre)
                    || armor.genre == DeviceId::UNKNOWN;
                // 过滤掉绿灯之上的前哨站、基地和未知装甲板（图像坐标系y向下为正）
                if (outpost_interference && armor.center.y < threshold_y) continue;
                filtered.push_back(armor);
            }
        }

        return Identifier::Result {
            .armors      = std::move(filtered),
            .green_light = locator_result.green_light,
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
