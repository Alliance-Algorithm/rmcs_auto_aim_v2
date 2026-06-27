#include "identifier.hpp"

#include "module/identifier/adjacency_lightbar.hpp"
#include "module/identifier/armor_detection.hpp"
#include "module/identifier/green_light.hpp"
#include "module/identifier/lightbar.hpp"

#include "utility/image/image.details.hpp"
#include "utility/math/angle.hpp"
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
        auto robots  = std::unordered_map<ArmorGenre, std::vector<const Armor2d*>> { };

        for (const auto& armor : detected) {
            if (armor.genre == DeviceId::OUTPOST) {
                outpost.push_back(armor);
            } else if (armor.genre == DeviceId::BASE) {
                base.push_back(armor);
            } else {
                robots[armor.genre].push_back(&armor);
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
                const auto is_building
                    = (armor.genre == ArmorGenre::OUTPOST) || (armor.genre == ArmorGenre::BASE);
                return is_building && armor.bl.y < 1. * y;
            };
            std::erase_if(detected, pred);
        }
        result.armors = detected;

        // 邻侧灯条：单装甲板机器人 → 扩展 ROI + 识别
        {
            constexpr auto kHorizontalCoeff  = 2.5;
            constexpr auto kVerticalConstant = 40;
            constexpr auto kGap              = 10;

            const auto& mat         = src.details().mat;
            const auto image_bounds = cv::Rect2i { 0, 0, mat.cols, mat.rows };

            for (const auto& [genre, armors] : robots) {
                if (armors.size() != 1) continue;

                const auto& armor = *armors[0];
                const auto min_x  = std::min({ armor.tl.x, armor.tr.x, armor.br.x, armor.bl.x });
                const auto max_x  = std::max({ armor.tl.x, armor.tr.x, armor.br.x, armor.bl.x });
                const auto min_y  = std::min({ armor.tl.y, armor.tr.y, armor.br.y, armor.bl.y });
                const auto max_y  = std::max({ armor.tl.y, armor.tr.y, armor.br.y, armor.bl.y });
                const auto width  = std::abs(armor.tr.x - armor.tl.x);

                const auto h_extend = static_cast<int>(width * kHorizontalCoeff);
                const auto top      = static_cast<int>(min_y - kVerticalConstant);
                const auto height   = static_cast<int>(max_y - min_y + 2 * kVerticalConstant);

                const auto camp = armor_color2camp_color(armor.color);

                const auto rois = std::array {
                    cv::Rect2i { static_cast<int>(min_x) - h_extend - kGap, top, h_extend, height },
                    cv::Rect2i { static_cast<int>(max_x) + kGap, top, h_extend, height },
                };
                for (const auto& roi : rois) {

                    const auto clipped = roi & image_bounds;
                    if (clipped.width <= 0 || clipped.height <= 0) continue;

                    result.areas.push_back(clipped);

                    auto finder = LightbarFinder { };
                    {
                        finder.input.source = mat(clipped).clone();
                        finder.input.color  = camp;
                        if (!finder.solve()) continue;
                    }
                    { // 长度筛选
                        const auto armor_length
                            = std::hypot(armor.tl.x - armor.bl.x, armor.tl.y - armor.bl.y);
                        const auto detected_length = std::hypot(
                            finder.result.upper.x - finder.result.lower.x,
                            finder.result.upper.y - finder.result.lower.y);
                        if (std::abs(detected_length - armor_length) / armor_length > 0.2) continue;
                    }
                    { // 角度筛选
                        const auto armor_direction
                            = cv::Point2d { armor.tl.x - armor.bl.x, armor.tl.y - armor.bl.y };
                        const auto detected_direction = cv::Point2d {
                            static_cast<double>(finder.result.upper.x - finder.result.lower.x),
                            static_cast<double>(finder.result.upper.y - finder.result.lower.y),
                        };
                        const auto armor_angle = std::atan2(armor_direction.y, armor_direction.x);
                        const auto detected_angle
                            = std::atan2(detected_direction.y, detected_direction.x);
                        const auto angle_diff
                            = std::abs(util::normalize_angle(detected_angle - armor_angle));
                        if (angle_diff > util::deg2rad(20.0)) continue;
                    }
                    result.lightbars.push_back({
                        .genre = genre,
                        .color = armor.color,
                        .upper = Point2d {
                            static_cast<double>(finder.result.upper.x + clipped.x),
                            static_cast<double>(finder.result.upper.y + clipped.y),
                        },
                        .lower = Point2d {
                            static_cast<double>(finder.result.lower.x + clipped.x),
                            static_cast<double>(finder.result.lower.y + clipped.y),
                        },
                    });
                }
            }
        }

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
