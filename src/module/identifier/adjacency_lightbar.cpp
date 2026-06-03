#include "adjacency_lightbar.hpp"
#include "module/identifier/lightbar.hpp"
#include "utility/image/image.details.hpp"
#include "utility/math/conversion.hpp"
#include "utility/math/outpost.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <vector>

#include <eigen3/Eigen/Geometry>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace rmcs {

struct AdjacencyLightbarFinder::Impl {
    struct DetectArea {
        cv::Rect2i roi;
        cv::Point2f near_upper;
        cv::Point2f near_lower;
        cv::Point2f away_upper;
        cv::Point2f away_lower;
    };
    std::array<DetectArea, 2> areas { };

    util::CameraFeature camera_feature { };
    util::NeighborBarSolution neighbor_solution { };

    std::optional<Lightbar> detected { };

    auto set_camera_feature(const util::CameraFeature& feature) { camera_feature = feature; }

    auto find(const Image& image, const Armor2D& armor2d, const Armor3D& armor3d)
        -> std::optional<Lightbar> {

        detected.reset();

        if (image.details().mat.empty()) return std::nullopt;

        const auto t = armor3d.translation.make<Eigen::Vector3d>();
        const auto q = armor3d.orientation.make<Eigen::Quaterniond>();

        const auto lpoint = Eigen::Vector3d { t + q * Eigen::Vector3d::UnitY() };
        const auto rpoint = Eigen::Vector3d { t + q * -Eigen::Vector3d::UnitY() };

        const auto pose_right = lpoint.norm() < rpoint.norm();
        const auto find_right = !pose_right;

        auto& solution          = neighbor_solution;
        solution.input.source   = armor3d;
        solution.input.in_right = find_right;
        solution.solve();

        // 开始选取 ROI，识别灯条
        const auto& mat = image.details().mat;

        const auto distance = armor3d.translation.make<Eigen::Vector3d>().norm();
        const auto margin_x = std::max(12, static_cast<int>(120.0 / distance + 32.0));
        const auto margin_y = std::max(12, static_cast<int>(090.0 / distance + 24.0));
        const auto camp     = armor_color2camp_color(armor3d.color);

        auto removed_mask =
            cv::boundingRect(std::vector { armor2d.tl, armor2d.tr, armor2d.br, armor2d.bl });
        removed_mask.x -= 5;
        removed_mask.y -= 5;
        removed_mask.width += 10;
        removed_mask.height += 10;
        removed_mask &= cv::Rect2i { 0, 0, mat.cols, mat.rows };

        const auto project_bar = [&](const util::NeighborBarSolution::Result::Bar& bar)
            -> std::optional<std::pair<cv::Point2f, cv::Point2f>> {
            auto segment_points = std::array<cv::Point3f, 2> { };
            for (std::size_t i = 0; i < segment_points.size(); ++i) {
                const auto point  = i == 0 ? bar.first : bar.second;
                const auto p      = util::ros2opencv_position(point.make<Eigen::Vector3d>());
                segment_points[i] = cv::Point3f(
                    static_cast<float>(p[0]), static_cast<float>(p[1]), static_cast<float>(p[2]));
            }

            auto projected = std::vector<cv::Point2f> { };
            cv::projectPoints(segment_points, cv::Vec3d { 0.0, 0.0, 0.0 },
                cv::Vec3d { 0.0, 0.0, 0.0 }, camera_feature.intrinsic(),
                camera_feature.distortion(), projected);
            if (projected.size() != 2) return std::nullopt;

            const auto upper = projected[0];
            const auto lower = projected[1];

            return std::pair { upper, lower };
        };

        for (auto&& [area, is_upper] : std::views::zip(areas, std::array { true, false })) {
            const auto& near_bar =
                is_upper ? solution.result.upper_near : solution.result.lower_near;
            const auto& away_bar =
                is_upper ? solution.result.upper_away : solution.result.lower_away;

            const auto near_result = project_bar(near_bar);
            if (!near_result) continue;

            const auto [near_upper, near_lower] = *near_result;

            const auto center = (near_upper + near_lower) * 0.5f;
            const auto roi    = cv::Rect2i {
                static_cast<int>(center.x) - margin_x,
                static_cast<int>(center.y) - margin_y,
                margin_x * 2,
                margin_y * 2,
            } & cv::Rect2i { 0, 0, mat.cols, mat.rows };

            const auto away_result = project_bar(away_bar);
            const auto [away_upper, away_lower] =
                away_result ? *away_result : std::pair { near_upper, near_lower };

            area = DetectArea {
                .roi        = roi,
                .near_upper = near_upper,
                .near_lower = near_lower,
                .away_upper = away_upper,
                .away_lower = away_lower,
            };
        }

        // 遍历 areas，检测灯条
        for (auto&& [is_upper, area] : std::views::zip(std::array { true, false }, areas)) {
            if (area.roi.width <= 0 || area.roi.height <= 0) continue;

            const auto roi_mat      = mat(area.roi).clone();
            const auto self_overlap = removed_mask & area.roi;
            if (self_overlap.area() > 0) {
                const auto local_overlap = cv::Rect2i {
                    self_overlap.x - area.roi.x,
                    self_overlap.y - area.roi.y,
                    self_overlap.width,
                    self_overlap.height,
                };
                roi_mat(local_overlap).setTo(cv::Scalar::all(0));
            }

            auto finder = LightbarFinder { };
            {
                auto& input = finder.input;

                input.source = roi_mat;
                input.color  = camp;

                input.predicted_upper = {
                    static_cast<int>(std::lround(area.near_upper.x - 1.0 * area.roi.x)),
                    static_cast<int>(std::lround(area.near_upper.y - 1.0 * area.roi.y)),
                };
                input.predicted_lower = {
                    static_cast<int>(std::lround(area.near_lower.x - 1.0 * area.roi.x)),
                    static_cast<int>(std::lround(area.near_lower.y - 1.0 * area.roi.y)),
                };
                if (!finder.solve()) continue;

                auto& result = finder.result;

                /// @NOTE:
                /// @TODO: 写一个 NOTE
                ///
                const auto length_detected = cv::norm(result.upper - result.lower);
                const auto length_predict  = cv::norm(area.near_upper - area.near_lower);

                if (length_detected > 1e-6) {
                    const auto scale  = length_predict / length_detected;
                    const auto center = (result.upper + result.lower) * 0.5f;
                    result.upper      = center + (result.upper - center) * scale;
                    result.lower      = center + (result.lower - center) * scale;
                }

                detected = Lightbar {
                    .upper =
                        Point2d {
                            1. * (result.upper.x + area.roi.x),
                            1. * (result.upper.y + area.roi.y),
                        },
                    .lower =
                        Point2d {
                            1. * (result.lower.x + area.roi.x),
                            1. * (result.lower.y + area.roi.y),
                        },
                    .is_right = find_right,
                    .is_upper = is_upper,
                };
                break;
            }
        }
        return detected;
    }

    auto draw_roi(Image& image) const -> void {
        static const auto kAreaColor   = cv::Scalar { 000, 255, 000 };
        static const auto kNearColor   = cv::Scalar { 255, 255, 255 };
        static const auto kAwayColor   = cv::Scalar { 128, 128, 128 };
        static const auto kCenterColor = cv::Scalar { 255, 255, 255 };

        auto& mat = image.details().mat;
        if (mat.empty()) return;

        const auto center_3d  = neighbor_solution.result.center.make<Eigen::Vector3d>();
        const auto center_ocv = util::ros2opencv_position(center_3d);

        auto center_projected = std::vector<cv::Point2f> { };
        cv::projectPoints(
            std::vector {
                cv::Point3f {
                    static_cast<float>(center_ocv.x()),
                    static_cast<float>(center_ocv.y()),
                    static_cast<float>(center_ocv.z()),
                },
            },
            cv::Vec3d { 0, 0, 0 }, cv::Vec3d { 0, 0, 0 }, camera_feature.intrinsic(),
            camera_feature.distortion(), center_projected);

        if (!center_projected.empty()) {
            cv::circle(mat, center_projected[0], 2, kCenterColor, -1, cv::LINE_AA);
        }

        for (const auto& area : areas) {
            cv::rectangle(mat, area.roi, kAreaColor, 1, cv::LINE_AA);
            cv::circle(mat, area.near_upper, 1, kNearColor, -1, cv::LINE_AA);
            cv::circle(mat, area.near_lower, 1, kNearColor, -1, cv::LINE_AA);
            cv::circle(mat, area.away_upper, 1, kAwayColor, -1, cv::LINE_AA);
            cv::circle(mat, area.away_lower, 1, kAwayColor, -1, cv::LINE_AA);
        }
    }

    auto draw_lightbar(Image& image) -> void {
        auto& mat = image.details().mat;
        if (mat.empty()) return;
        if (!detected.has_value()) return;

        cv::circle(
            mat, detected->upper.make<cv::Point2f>(), 2, cv::Scalar { 0, 0, 255 }, -1, cv::LINE_AA);
        cv::circle(
            mat, detected->lower.make<cv::Point2f>(), 2, cv::Scalar { 0, 0, 255 }, -1, cv::LINE_AA);
    }
};

auto AdjacencyLightbarFinder::set_camera_feature(const util::CameraFeature& feature) -> void {
    return pimpl->set_camera_feature(feature);
}

auto AdjacencyLightbarFinder::find(
    const Image& image, const Armor2D& armor2d, const Armor3D& armor3d) -> std::optional<Lightbar> {
    return pimpl->find(image, armor2d, armor3d);
}

auto AdjacencyLightbarFinder::draw_roi(Image& image) -> void { return pimpl->draw_roi(image); }

auto AdjacencyLightbarFinder::draw_lightbar(Image& image) -> void {
    return pimpl->draw_lightbar(image);
}

AdjacencyLightbarFinder::AdjacencyLightbarFinder() noexcept
    : pimpl { std::make_unique<Impl>() } { }

AdjacencyLightbarFinder::~AdjacencyLightbarFinder() noexcept = default;

} // namespace rmcs
