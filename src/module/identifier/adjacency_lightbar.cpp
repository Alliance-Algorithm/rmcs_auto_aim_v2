#include "adjacency_lightbar.hpp"
#include "module/identifier/lightbar.hpp"
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
        cv::Rect2i area;
        Lightbar2d near;
        Lightbar2d away;
    };
    std::array<DetectArea, 2> areas { };

    util::CameraFeature camera_feature { };
    util::NeighborBarSolution neighbor_solution { };
    double armor_thickness { 0 };

    std::optional<Lightbar2d> detected { };

    auto set_camera_feature(const util::CameraFeature& feature) { camera_feature = feature; }

    auto set_armor_thickness(double thickness) { armor_thickness = thickness; }

    auto find(const cv::Mat& mat, const Armor2d& armor2d, const Armor3d& armor3d)
        -> std::optional<Result> {

        detected.reset();

        if (mat.empty()) return std::nullopt;

        const auto t = armor3d.translation.make<Eigen::Vector3d>();
        const auto q = armor3d.orientation.make<Eigen::Quaterniond>();

        const auto lpoint = Eigen::Vector3d { t + q * Eigen::Vector3d::UnitY() };
        const auto rpoint = Eigen::Vector3d { t + q * -Eigen::Vector3d::UnitY() };

        const auto pose_right = lpoint.norm() < rpoint.norm();
        const auto find_right = !pose_right;

        auto& solution                 = neighbor_solution;
        solution.input.source          = armor3d;
        solution.input.in_right        = find_right;
        solution.input.armor_thickness = armor_thickness;
        solution.solve();

        // 开始选取 ROI，识别灯条
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

        const auto project_bar = [&](const Lightbar3d& bar) -> std::optional<Lightbar2d> {
            auto segment_points = std::array<cv::Point3f, 2> { };
            for (std::size_t i = 0; i < segment_points.size(); ++i) {
                const auto point  = i == 0 ? bar.upper : bar.lower;
                const auto p      = util::ros2opencv_position(point.make<Eigen::Vector3d>());
                segment_points[i] = cv::Point3f(
                    static_cast<float>(p[0]), static_cast<float>(p[1]), static_cast<float>(p[2]));
            }

            auto projected = std::vector<cv::Point2f> { };
            cv::projectPoints(segment_points, cv::Vec3d { 0.0, 0.0, 0.0 },
                cv::Vec3d { 0.0, 0.0, 0.0 }, camera_feature.intrinsic(),
                camera_feature.distortion(), projected);
            if (projected.size() != 2) return std::nullopt;

            return Lightbar2d {
                .color = bar.color,
                .upper = Point2d { projected[0] },
                .lower = Point2d { projected[1] },
            };
        };

        for (auto&& [area, is_upper] : std::views::zip(areas, std::array { true, false })) {
            const auto& near_bar =
                is_upper ? solution.result.upper_near : solution.result.lower_near;
            const auto& away_bar =
                is_upper ? solution.result.upper_away : solution.result.lower_away;

            const auto near_result = project_bar(near_bar);
            if (!near_result) continue;

            const auto& near_lightbar = *near_result;

            const auto center = near_lightbar.upper.make<cv::Point2f>() * 0.5f
                + near_lightbar.lower.make<cv::Point2f>() * 0.5f;
            const auto roi = cv::Rect2i {
                static_cast<int>(center.x) - margin_x,
                static_cast<int>(center.y) - margin_y,
                margin_x * 2,
                margin_y * 2,
            } & cv::Rect2i { 0, 0, mat.cols, mat.rows };

            const auto away_result    = project_bar(away_bar);
            const auto& away_lightbar = away_result ? *away_result : near_lightbar;

            area = DetectArea {
                .area = roi,
                .near = near_lightbar,
                .away = away_lightbar,
            };
        }

        // 遍历 areas，检测灯条
        for (auto&& [is_upper, area] : std::views::zip(std::array { true, false }, areas)) {
            if (area.area.width <= 0 || area.area.height <= 0) continue;

            const auto roi_mat      = mat(area.area).clone();
            const auto self_overlap = removed_mask & area.area;
            if (self_overlap.area() > 0) {
                const auto local_overlap = cv::Rect2i {
                    self_overlap.x - area.area.x,
                    self_overlap.y - area.area.y,
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
                    static_cast<int>(std::lround(area.near.upper.x - 1.0 * area.area.x)),
                    static_cast<int>(std::lround(area.near.upper.y - 1.0 * area.area.y)),
                };
                input.predicted_lower = {
                    static_cast<int>(std::lround(area.near.lower.x - 1.0 * area.area.x)),
                    static_cast<int>(std::lround(area.near.lower.y - 1.0 * area.area.y)),
                };
                if (!finder.solve()) continue;

                auto& result = finder.result;

                /// @NOTE:
                ///  Lightbar 不是一个理想的长方体或者圆柱体，随着 Yaw 的增大，其长度会
                ///  发生变化，影响距离的估计
                const auto length_detected = cv::norm(result.upper - result.lower);
                const auto length_predict  = std::hypot(
                    area.near.upper.x - area.near.lower.x, area.near.upper.y - area.near.lower.y);

                if (length_detected > 1e-6) {
                    const auto scale  = length_predict / length_detected;
                    const auto center = (result.upper + result.lower) * 0.5f;
                    result.upper      = center + (result.upper - center) * scale;
                    result.lower      = center + (result.lower - center) * scale;
                }

                detected = Lightbar2d {
                    .color = armor3d.color,
                    .upper =
                        Point2d {
                            1. * (result.upper.x + area.area.x),
                            1. * (result.upper.y + area.area.y),
                        },
                    .lower =
                        Point2d {
                            1. * (result.lower.x + area.area.x),
                            1. * (result.lower.y + area.area.y),
                        },
                    .is_upper = is_upper,
                    .is_right = find_right,
                };
                break;
            }
        }

        auto result = Result { };

        if (detected.has_value()) {
            auto found       = *detected;
            found.draw_color = { 0, 0, 255 };
            result.found.push_back(found);
        }
        for (const auto& area : areas) {
            auto near       = area.near;
            near.draw_color = { 255, 255, 255 };

            auto away       = area.away;
            away.draw_color = { 128, 128, 128 };

            result.predicted_near.push_back(near);
            result.predicted_away.push_back(away);
        }
        for (const auto& area : areas) {
            result.areas.push_back(area.area);
        }

        // 计算中心点的投影
        auto center_projected = std::vector<cv::Point2f> { };
        {
            const auto center_3d = neighbor_solution.result.center.make<Eigen::Vector3d>();
            const auto center_cv = util::ros2opencv_position(center_3d);

            cv::projectPoints(
                std::vector {
                    cv::Point3f {
                        static_cast<float>(center_cv.x()),
                        static_cast<float>(center_cv.y()),
                        static_cast<float>(center_cv.z()),
                    },
                },
                cv::Vec3d { 0, 0, 0 }, cv::Vec3d { 0, 0, 0 }, camera_feature.intrinsic(),
                camera_feature.distortion(), center_projected);
        }
        if (!center_projected.empty()) {
            result.center = center_projected[0];
        }

        return result;
    }
};

auto AdjacencyLightbarFinder::set_camera_feature(const util::CameraFeature& feature) -> void {
    return pimpl->set_camera_feature(feature);
}

auto AdjacencyLightbarFinder::set_armor_thickness(double thickness) -> void {
    return pimpl->set_armor_thickness(thickness);
}

auto AdjacencyLightbarFinder::find(
    const cv::Mat& image, const Armor2d& armor2d, const Armor3d& armor3d) -> std::optional<Result> {
    return pimpl->find(image, armor2d, armor3d);
}

AdjacencyLightbarFinder::AdjacencyLightbarFinder() noexcept
    : pimpl { std::make_unique<Impl>() } { }

AdjacencyLightbarFinder::~AdjacencyLightbarFinder() noexcept = default;

} // namespace rmcs
