#include "adjacency_lightbar.hpp"
#include "module/identifier/lightbar.hpp"

#include "utility/image/image.details.hpp"
#include "utility/math/angle.hpp"
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
    struct CandidateRoi {
        cv::Rect2i roi;
        cv::Point2f higher;
        cv::Point2f lowwer;
    };
    std::vector<CandidateRoi> candidates { };
    std::optional<Lightbar> detected_lightbar { };

    util::CameraFeature camera_feature { };
    bool has_camera_feature { false };

    auto set_camera_feature(const util::CameraFeature& feature) -> void {
        camera_feature     = feature;
        has_camera_feature = true;
    }

    auto find(const Image& image, const Armor2D&, const Armor3D& armor3d)
        -> std::optional<Lightbar> {

        candidates.clear();
        detected_lightbar.reset();

        // 基础输入校验。
        if (!has_camera_feature) return std::nullopt;
        if (image.details().mat.empty()) return std::nullopt;

        const auto yaw          = util::eulers(armor3d.orientation.make<Eigen::Quaterniond>())[0];
        const auto prefer_right = yaw >= 0.0;

        auto right_solution           = util::NeighborBarSolution { };
        right_solution.input.source   = armor3d;
        right_solution.input.in_right = true;
        right_solution.solve();

        auto left_solution           = util::NeighborBarSolution { };
        left_solution.input.source   = armor3d;
        left_solution.input.in_right = false;
        left_solution.solve();

        const auto& preferred_solution = prefer_right ? right_solution : left_solution;
        const auto& secondary_solution = prefer_right ? left_solution : right_solution;

        const auto preferred_yaw = prefer_right
            ? util::normalize_angle(yaw + 2.0 * std::numbers::pi / 3.0)
            : util::normalize_angle(yaw - 2.0 * std::numbers::pi / 3.0);
        const auto secondary_yaw = prefer_right
            ? util::normalize_angle(yaw - 2.0 * std::numbers::pi / 3.0)
            : util::normalize_angle(yaw + 2.0 * std::numbers::pi / 3.0);

        struct CandidateBar {
            std::pair<Point3d, Point3d> bar;
            double neighbor_yaw;
        };
        const auto bars = std::array {
            CandidateBar { preferred_solution.result.bars[0], preferred_yaw },
            CandidateBar { preferred_solution.result.bars[1], preferred_yaw },
            CandidateBar { secondary_solution.result.bars[0], secondary_yaw },
            CandidateBar { secondary_solution.result.bars[1], secondary_yaw },
        };

        const auto& mat     = image.details().mat;
        const auto distance = armor3d.translation.make<Eigen::Vector3d>().norm();
        const auto margin_x = std::max(12, static_cast<int>(120.0 / distance + 8.0));
        const auto margin_y = std::max(12, static_cast<int>(090.0 / distance + 6.0));
        const auto camp     = armor_color2camp_color(armor3d.color);

        for (const auto& candidate : bars) {
            // 重投影候选灯条的两个端点。
            auto segment_points = std::array<cv::Point3f, 2> { };
            for (std::size_t i = 0; i < segment_points.size(); ++i) {
                const auto point  = i == 0 ? candidate.bar.first : candidate.bar.second;
                const auto p      = util::ros2opencv_position(point.make<Eigen::Vector3d>());
                segment_points[i] = cv::Point3f(
                    static_cast<float>(p[0]), static_cast<float>(p[1]), static_cast<float>(p[2]));
            }

            auto projected_segment = std::vector<cv::Point2f> { };
            cv::projectPoints(segment_points, cv::Vec3d { 0.0, 0.0, 0.0 },
                cv::Vec3d { 0.0, 0.0, 0.0 }, camera_feature.intrinsic(),
                camera_feature.distortion(), projected_segment);
            if (projected_segment.size() != 2) continue;

            auto top    = projected_segment[0];
            auto bottom = projected_segment[1];
            if (top.y > bottom.y) std::swap(top, bottom);

            // 围绕预测灯条生成候选 ROI。
            auto roi = cv::boundingRect(std::vector<cv::Point2f> { top, bottom });
            roi.x -= margin_x;
            roi.y -= margin_y;
            roi.width += margin_x * 3;
            roi.height += margin_y * 3;

            roi &= cv::Rect2i { 0, 0, mat.cols, mat.rows };
            if (roi.width <= 0 || roi.height <= 0) continue;

            auto finder                   = LightbarFinder { };
            finder.input.roi              = mat(roi);
            finder.input.color            = camp;
            finder.input.predicted_point1 = { static_cast<int>(std::lround(top.x - 1.0 * roi.x)),
                static_cast<int>(std::lround(top.y - 1.0 * roi.y)) };
            finder.input.predicted_point2 = { static_cast<int>(std::lround(bottom.x - 1.0 * roi.x)),
                static_cast<int>(std::lround(bottom.y - 1.0 * roi.y)) };
            if (!finder.solve()) continue;

            auto precise_top    = cv::Point2f { static_cast<float>(finder.result.point1.x + roi.x),
                static_cast<float>(finder.result.point1.y + roi.y) };
            auto precise_bottom = cv::Point2f { static_cast<float>(finder.result.point2.x + roi.x),
                static_cast<float>(finder.result.point2.y + roi.y) };

            candidates.push_back({ roi, precise_top, precise_bottom });
        }

        if (candidates.empty()) return std::nullopt;

        detected_lightbar = Lightbar {
            .point1 = Point2d { candidates.front().higher },
            .point2 = Point2d { candidates.front().lowwer },
        };
        return detected_lightbar;
    }

    auto draw_roi(Image& image) -> void {
        auto& mat = image.details().mat;
        if (mat.empty()) return;

        static const auto kColors = std::array {
            cv::Scalar { 0, 255, 0 },
            cv::Scalar { 0, 255, 255 },
        };

        for (std::size_t i = 0; i < candidates.size(); ++i) {
            const auto& candidate = candidates[i];
            const auto& color     = kColors[i % kColors.size()];

            cv::rectangle(mat, candidate.roi, color, 1, cv::LINE_AA);
            cv::circle(mat, candidate.higher, 2, color, -1, cv::LINE_AA);
            cv::circle(mat, candidate.lowwer, 2, color, -1, cv::LINE_AA);
        }
    }

    auto draw_lightbar(Image& image) -> void {
        auto& mat = image.details().mat;
        if (mat.empty()) return;
        if (!detected_lightbar.has_value()) return;

        cv::circle(mat, detected_lightbar->point1.make<cv::Point2f>(), 3, cv::Scalar { 0, 0, 255 },
            -1, cv::LINE_AA);
        cv::circle(mat, detected_lightbar->point2.make<cv::Point2f>(), 3, cv::Scalar { 0, 0, 255 },
            -1, cv::LINE_AA);
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
