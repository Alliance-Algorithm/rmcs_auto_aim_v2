#include "detector.hpp"
#include "module/detector/armor_detection.hpp"
#include "module/detector/green_light.hpp"
#include "module/detector/lightbar.hpp"
#include "module/detector/rune.hpp"
#include "utility/math/angle.hpp"
#include "utility/math/corners_optimizor.hpp"
#include "utility/robot/armor.hpp"

#include <optional>
#include <span>
#include <vector>

using namespace rmcs;
using namespace rmcs::kernel;
using namespace rmcs::util;
using namespace rmcs::detector;

struct Detector::Impl {
    CameraFeature cam;
    CampColor color;

    ArmorDetection armor_detection;
    GreenLightFinder green_light_finder;

    RuneDetector rune_detector;

    static auto find_lightbar(const cv::Mat& mat, const Armor2d& armor, Result& result) {
        const auto bounds = cv::Rect2i { 0, 0, mat.cols, mat.rows };

        const auto min_x = std::min({ armor.tl.x, armor.tr.x, armor.br.x, armor.bl.x });
        const auto max_x = std::max({ armor.tl.x, armor.tr.x, armor.br.x, armor.bl.x });
        const auto min_y = std::min({ armor.tl.y, armor.tr.y, armor.br.y, armor.bl.y });
        const auto max_y = std::max({ armor.tl.y, armor.tr.y, armor.br.y, armor.bl.y });

        const auto aw = std::abs(max_x - min_x);
        const auto ah = std::abs(max_y - min_y);

        const auto top = static_cast<int>(min_y - ah * 2.);
        const auto gap = static_cast<int>(aw * 1.0);

        const auto rh = static_cast<int>(aw * 2.5);
        const auto rw = static_cast<int>(aw * 1.5);

        const auto point = [](auto x, auto y) {
            return cv::Point2i {
                static_cast<int>(std::round(x)),
                static_cast<int>(std::round(y)),
            };
        };
        const auto rois = std::array {
            cv::Rect2i { point(min_x - 1. * gap - 1. * rw, top), cv::Size2i { rw, rh } },
            cv::Rect2i { point(max_x + 1. * gap - 0. * rw, top), cv::Size2i { rw, rh } },
        };

        for (const auto& roi : rois) {
            const auto clipped = roi & bounds;
            if (clipped.width <= 0 || clipped.height <= 0) continue;

            result.areas.push_back(clipped);

            auto finder = LightbarFinder { };
            {
                const auto camp = armor_color2camp_color(armor.color);

                finder.input.source = mat(clipped).clone();
                finder.input.color  = camp;
                if (!finder.solve()) continue;
            }
            { // 长度筛选
                const auto armor_length = std::max<double>(
                    std::hypot(armor.tl.x - armor.bl.x, armor.tl.y - armor.bl.y), 0.01);
                const auto detected_length =
                    std::max<double>(std::hypot(finder.result.upper.x - finder.result.lower.x,
                                         finder.result.upper.y - finder.result.lower.y),
                        0.01);
                if (std::abs(detected_length - armor_length) / armor_length > 0.2) continue;
            }
            { // 角度筛选
                const auto armor_direction =
                    cv::Point2d { armor.tl.x - armor.bl.x, armor.tl.y - armor.bl.y };
                const auto detected_direction = cv::Point2d {
                    static_cast<double>(finder.result.upper.x - finder.result.lower.x),
                    static_cast<double>(finder.result.upper.y - finder.result.lower.y),
                };
                const auto armor_angle    = std::atan2(armor_direction.y, armor_direction.x);
                const auto detected_angle = std::atan2(detected_direction.y, detected_direction.x);
                const auto angle_diff =
                    std::abs(util::normalize_angle(detected_angle - armor_angle));
                if (angle_diff > util::deg2rad(20.0)) continue;
            }
            result.lightbars.push_back({
                .genre = armor.genre,
                .color = armor.color,
                .upper =
                    Point2d {
                        static_cast<double>(finder.result.upper.x + clipped.x),
                        static_cast<double>(finder.result.upper.y + clipped.y),
                    },
                .lower =
                    Point2d {
                        static_cast<double>(finder.result.lower.x + clipped.x),
                        static_cast<double>(finder.result.lower.y + clipped.y),
                    },
            });
        }
    }

    auto initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
        auto armor_result = armor_detection.initialize(yaml);
        if (!armor_result.has_value()) return std::unexpected { armor_result.error() };

        auto locator_result = green_light_finder.initialize(yaml["green_light_filter"]);
        if (!locator_result.has_value()) return std::unexpected { locator_result.error() };

        return { };
    }

    auto detect(const cv::Mat& mat) noexcept -> Result {
        auto result = Result { };

        const auto elements = rune_detector.detect(mat);
        {
            result.icons     = elements.icons;
            result.bullseyes = elements.bullseyes;
        }

        auto detected = armor_detection.sync_detect(mat);
        if (detected.empty()) return result;

        std::erase_if(
            detected, [](const Armor2d& armor) { return armor.genre == ArmorGenre::UNKNOWN; });

        util::optimize_corners(mat, detected);

        auto outpost = Armor2ds { };
        auto base    = Armor2ds { };

        // 使用 map 是为了统计每个 Genre 的装甲板数量，只对特定数量的种类进行灯条识别
        auto robots = std::map<ArmorGenre, std::vector<const Armor2d*>> { };

        for (const auto& armor : detected) {
            if (armor.genre == DeviceId::OUTPOST) {
                outpost.push_back(armor);
            } else if (armor.genre == DeviceId::BASE) {
                base.push_back(armor);
            } else {
                robots[armor.genre].push_back(&armor);
            }
        }

        /// @NOTE:
        ///  - 前哨站与基地的绿灯不会同时亮起，所以我们只需要维护一个绿灯即可，
        ///  再把高于绿灯高度的建筑类型的装甲板滤除即可，即使将前哨站误识别
        ///  成了基地（这相当容易），也不影响滤除的效果
        ///  - 这里分开识别是考虑到：如果同时识别到了真正的基地和前哨站，其外
        ///  包矩形将会过于大
        auto& finder = green_light_finder;
        if (!base.empty()) {
            if (auto ret = finder.locate(mat, base); ret.green_light) {
                result.green_light = ret.green_light;
                result.areas.push_back(*ret.detect_roi);
            }
        }
        if (!outpost.empty()) {
            if (auto ret = finder.locate(mat, outpost); ret.green_light) {
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

        // 邻侧灯条：单装甲板机器人 → 扩展 ROI + 识别
        for (const auto& [_, armors] : robots) {
            if (armors.size() == 1) {
                find_lightbar(mat, *armors[0], result);
            }
        }

        return result;
    }
};

auto Detector::initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
    return pimpl->initialize(yaml);
}

auto Detector::update_detect_color(CampColor color) -> void {
    pimpl->color                      = color;
    pimpl->rune_detector.config.color = color;
}

auto Detector::update_camera(const std::array<double, 9>& var) noexcept -> void {
    pimpl->rune_detector.config.cam.from(var);
}
auto Detector::update_camera(const std::array<double, 5>& var) noexcept -> void {
    pimpl->rune_detector.config.cam.from(var);
}

auto Detector::detect(const cv::Mat& src) noexcept -> Result { return pimpl->detect(src); }

Detector::Detector() noexcept
    : pimpl { std::make_unique<Impl>() } { }

Detector::~Detector() noexcept = default;
