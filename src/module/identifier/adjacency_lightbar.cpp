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
    struct DetectResult {
        cv::Rect2i roi;

        cv::Point2f predicted_upper;
        cv::Point2f predicted_lower;

        cv::Point2f upper;
        cv::Point2f lower;

        bool is_right { false };
        bool is_upper { false };

        bool detected { false };
    };
    std::vector<DetectResult> results { };

    util::CameraFeature camera_feature { };
    std::optional<Lightbar> detected_lightbar { };

    auto set_camera_feature(const util::CameraFeature& feature) { camera_feature = feature; }

    auto find(const Image& image, const Armor2D& armor2d, const Armor3D& armor3d)
        -> std::optional<Lightbar> {

        results.clear();
        detected_lightbar.reset();

        if (image.details().mat.empty()) return std::nullopt;

        const auto t = armor3d.translation.make<Eigen::Vector3d>();
        const auto q = armor3d.orientation.make<Eigen::Quaterniond>();

        const auto lunit = Eigen::Vector3d { 0.0, +1.0, 0.0 };
        const auto runit = Eigen::Vector3d { 0.0, -1.0, 0.0 };

        const auto lpoint = Eigen::Vector3d { (t + q * lunit).eval() };
        const auto rpoint = Eigen::Vector3d { (t + q * runit).eval() };

        const auto lnorm = lpoint.x() * lpoint.x() + lpoint.y() * lpoint.y();
        const auto rnorm = rpoint.x() * rpoint.x() + rpoint.y() * rpoint.y();

        const auto in_right = rnorm < lnorm;

        auto solution           = util::NeighborBarSolution { };
        solution.input.source   = armor3d;
        solution.input.in_right = in_right;
        solution.solve();

        const auto upper_bar = solution.result.upper_near;
        const auto lower_bar = solution.result.lower_near;

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

        for (auto&& [bar, is_upper] :
            std::views::zip(std::array { upper_bar, lower_bar }, std::array { true, false })) {

            // 重投影候选灯条的两个端点。
            auto segment_points = std::array<cv::Point3f, 2> { };
            for (std::size_t i = 0; i < segment_points.size(); ++i) {
                const auto point  = i == 0 ? bar.first : bar.second;
                const auto p      = util::ros2opencv_position(point.make<Eigen::Vector3d>());
                segment_points[i] = cv::Point3f(
                    static_cast<float>(p[0]), static_cast<float>(p[1]), static_cast<float>(p[2]));
            }

            auto projected_segment = std::vector<cv::Point2f> { };
            cv::projectPoints(segment_points, cv::Vec3d { 0.0, 0.0, 0.0 },
                cv::Vec3d { 0.0, 0.0, 0.0 }, camera_feature.intrinsic(),
                camera_feature.distortion(), projected_segment);
            if (projected_segment.size() != 2) continue;

            auto upper_point = projected_segment[0];
            auto lower_point = projected_segment[1];
            if (upper_point.y > lower_point.y) std::swap(upper_point, lower_point);

            // 以两点中点为中心生成候选 ROI。
            auto center = (upper_point + lower_point) * 0.5f;
            auto roi    = cv::Rect2i {
                static_cast<int>(center.x) - margin_x,
                static_cast<int>(center.y) - margin_y,
                margin_x * 2,
                margin_y * 2,
            };

            roi &= cv::Rect2i { 0, 0, mat.cols, mat.rows };
            if (roi.width <= 0 || roi.height <= 0) continue;

            results.push_back(DetectResult {
                .roi = roi,

                .predicted_upper = upper_point,
                .predicted_lower = lower_point,

                .upper = upper_point,
                .lower = lower_point,

                .is_right = in_right,
                .is_upper = is_upper,

                .detected = false,
            });

            const auto roi_mat      = mat(roi).clone();
            const auto self_overlap = removed_mask & roi;
            if (self_overlap.area() > 0) {
                const auto local_overlap = cv::Rect2i {
                    self_overlap.x - roi.x,
                    self_overlap.y - roi.y,
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

                input.predicted_point1 = {
                    static_cast<int>(std::lround(upper_point.x - 1.0 * roi.x)),
                    static_cast<int>(std::lround(upper_point.y - 1.0 * roi.y)),
                };
                input.predicted_point2 = {
                    static_cast<int>(std::lround(lower_point.x - 1.0 * roi.x)),
                    static_cast<int>(std::lround(lower_point.y - 1.0 * roi.y)),
                };
                if (!finder.solve()) continue;
            }
            {
                auto& result = finder.result;

                results.back().upper = cv::Point2f {
                    static_cast<float>(result.point1.x + roi.x),
                    static_cast<float>(result.point1.y + roi.y),
                };
                results.back().lower = cv::Point2f {
                    static_cast<float>(result.point2.x + roi.x),
                    static_cast<float>(result.point2.y + roi.y),
                };
                results.back().detected = true;
            }
        }

        if (results.empty()) {
            return std::nullopt;
        }

        const auto detected = std::ranges::find_if(
            results, [](const DetectResult& candidate) { return candidate.detected; });
        if (detected == results.end()) {
            return std::nullopt;
        }

        detected_lightbar = Lightbar {
            .point1   = Point2d { detected->upper },
            .point2   = Point2d { detected->lower },
            .is_right = detected->is_right,
            .is_upper = detected->is_upper,
        };
        return detected_lightbar;
    }

    auto draw_roi(Image& image) -> void {
        auto& mat = image.details().mat;
        if (mat.empty()) return;

        static const auto kRoiColors = std::array {
            cv::Scalar { 0, 255, 0 },
            cv::Scalar { 0, 255, 255 },
        };
        static const auto kPredictedPointColor = cv::Scalar { 255, 255, 255 };

        for (std::size_t i = 0; i < results.size(); ++i) {
            const auto& candidate = results[i];
            const auto& color     = kRoiColors[i % kRoiColors.size()];

            cv::rectangle(mat, candidate.roi, color, 1, cv::LINE_AA);
            cv::circle(mat, candidate.predicted_upper, 1, kPredictedPointColor, -1, cv::LINE_AA);
            cv::circle(mat, candidate.predicted_lower, 1, kPredictedPointColor, -1, cv::LINE_AA);
        }
    }

    auto draw_lightbar(Image& image) -> void {
        auto& mat = image.details().mat;
        if (mat.empty()) return;
        if (!detected_lightbar.has_value()) return;

        cv::circle(mat, detected_lightbar->point1.make<cv::Point2f>(), 2, cv::Scalar { 0, 0, 255 },
            -1, cv::LINE_AA);
        cv::circle(mat, detected_lightbar->point2.make<cv::Point2f>(), 2, cv::Scalar { 0, 0, 255 },
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
