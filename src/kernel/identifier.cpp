#include "identifier.hpp"

#include "module/identifier/adjacency_lightbar.hpp"
#include "module/identifier/armor_detection.hpp"
#include "module/identifier/green_light.hpp"

#include "utility/math/corners_optimizor.hpp"
#include "utility/robot/armor.hpp"

#include <optional>
#include <span>
#include <vector>

using namespace rmcs;
using namespace rmcs::kernel;
using namespace rmcs::identifier;

struct Identifier::Impl {
    ArmorDetection armor_detection;
    GreenLightFinder green_light_finder;
    AdjacencyLightbarFinder adjacency_finder;

    auto initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
        auto armor_result = armor_detection.initialize(yaml);
        if (!armor_result.has_value()) return std::unexpected { armor_result.error() };

        auto locator_result = green_light_finder.initialize(yaml["green_light_filter"]);
        if (!locator_result.has_value()) return std::unexpected { locator_result.error() };

        return { };
    }

    auto identify(const Image& src) noexcept -> std::optional<Identifier::Result> {
        auto detected = armor_detection.sync_detect(src);
        if (detected.empty()) return std::nullopt;

        std::erase_if(
            detected, [](const Armor2d& armor) { return armor.genre == ArmorGenre::UNKNOWN; });

        util::optimize_corners(src, detected);

        auto outpost = Armor2ds { };
        auto base    = Armor2ds { };
        for (const auto& armor : detected) {
            if (armor.genre == DeviceId::OUTPOST) {
                outpost.push_back(armor);
            } else if (armor.genre == DeviceId::BASE) {
                base.push_back(armor);
            }
        }
        auto result = Result { };

        /// @NOTE:
        ///  - 前哨站与基地的绿灯不会同时亮起，所以我们只需要维护一个绿灯即可，
        ///  再把高于绿灯高度的建筑类型的装甲板滤除即可，即使将前哨站误识别
        ///  成了基地（这相当容易），也不影响滤除的效果
        ///  - 这里分开识别是考虑到：如果同时识别到了真正的基地和前哨站，其外
        ///  包矩形将会过于大
        auto& finder = green_light_finder;
        if (!base.empty()) {
            if (auto ret = finder.locate(src, base); ret.green_light) {
                result.green_light = ret.green_light;
                result.areas.push_back(*ret.detect_roi);
            }
        }
        if (!outpost.empty()) {
            if (auto ret = finder.locate(src, outpost); ret.green_light) {
                result.green_light = ret.green_light;
                result.areas.push_back(*ret.detect_roi);
            }
        }
        if (result.green_light) {
            // 取左上角的点的 Y 值即可
            const auto y = result.green_light->y;

            const auto pred = [=](const Armor2d& armor) {
                const auto is_building =
                    (armor.genre == ArmorGenre::OUTPOST) || (armor.genre == ArmorGenre::BASE);
                return is_building && armor.bl.y < 1. * y;
            };
            std::erase_if(detected, pred);
        }
        result.armors = detected;

        return result;
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
