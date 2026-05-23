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
    struct CandidateRoi {
        cv::Rect2i roi;

        cv::Point2f predicted_upper;
        cv::Point2f predicted_lower;

        cv::Point2f upper;
        cv::Point2f lower;
        bool is_right { false };
        bool is_upper { false };
        bool detected { false };
    };
    std::vector<CandidateRoi> candidates { };
    std::optional<Lightbar> detected_lightbar { };

    util::CameraFeature camera_feature { };
    bool has_camera_feature { false };

    auto set_camera_feature(const util::CameraFeature& feature) -> void {
        camera_feature     = feature;
        has_camera_feature = true;
    }

    auto find(const Image& image, const Armor2D& armor2d, const Armor3D& armor3d)
        -> std::optional<Lightbar> {

        candidates.clear();
        detected_lightbar.reset();

        // 基础输入校验。
        if (!has_camera_feature) return std::nullopt;
        if (image.details().mat.empty()) return std::nullopt;

        const auto translation = armor3d.translation.make<Eigen::Vector3d>();
        const auto rotation    = armor3d.orientation.make<Eigen::Quaterniond>().toRotationMatrix();

        const auto lunit  = Eigen::Vector3d { 0.0, +1.0, 0.0 };
        const auto runit  = Eigen::Vector3d { 0.0, -1.0, 0.0 };
        const auto lpoint = Eigen::Vector3d { (translation + rotation * lunit).eval() };
        const auto rpoint = Eigen::Vector3d { (translation + rotation * runit).eval() };

        const auto lnorm = lpoint.x() * lpoint.x() + lpoint.y() * lpoint.y();
        const auto rnorm = rpoint.x() * rpoint.x() + rpoint.y() * rpoint.y();

        // The side closer to the origin in the ROS xy plane is treated as the incoming side.
        const auto prefer_right = rnorm < lnorm;

        auto solution           = util::NeighborBarSolution { };
        solution.input.source   = armor3d;
        solution.input.in_right = prefer_right;
        solution.solve();

        struct CandidateBar {
            std::pair<Point3d, Point3d> bar;
        };
        const auto bars = std::array {
            CandidateBar { solution.result.bars[0] },
            CandidateBar { solution.result.bars[1] },
        };
        const auto bar0_center_z = 0.5 * (bars[0].bar.first.z + bars[0].bar.second.z);
        const auto bar1_center_z = 0.5 * (bars[1].bar.first.z + bars[1].bar.second.z);

        const auto& mat     = image.details().mat;
        const auto distance = armor3d.translation.make<Eigen::Vector3d>().norm();
        const auto margin_x = std::max(12, static_cast<int>(120.0 / distance + 8.0));
        const auto margin_y = std::max(12, static_cast<int>(090.0 / distance + 6.0));
        const auto camp     = armor_color2camp_color(armor3d.color);

        auto self_roi = cv::boundingRect(
            std::vector<cv::Point2f> { armor2d.tl, armor2d.tr, armor2d.br, armor2d.bl });
        self_roi.x -= 5;
        self_roi.y -= 5;
        self_roi.width += 10;
        self_roi.height += 10;
        self_roi &= cv::Rect2i { 0, 0, mat.cols, mat.rows };

        for (std::size_t candidate_index = 0; candidate_index < bars.size(); ++candidate_index) {
            const auto& candidate = bars[candidate_index];
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

            const auto candidate_center_z = candidate_index == 0 ? bar0_center_z : bar1_center_z;
            const auto other_center_z     = candidate_index == 0 ? bar1_center_z : bar0_center_z;
            candidates.push_back({
                roi,
                top,
                bottom,
                top,
                bottom,
                prefer_right,
                candidate_center_z > other_center_z,
                false,
            });

            auto roi_mat            = mat(roi).clone();
            const auto self_overlap = self_roi & roi;
            if (self_overlap.area() > 0) {
                const auto local_overlap = cv::Rect2i {
                    self_overlap.x - roi.x,
                    self_overlap.y - roi.y,
                    self_overlap.width,
                    self_overlap.height,
                };
                roi_mat(local_overlap).setTo(cv::Scalar::all(0));
            }

            auto finder                   = LightbarFinder { };
            finder.input.roi              = roi_mat;
            finder.input.color            = camp;
            finder.input.predicted_point1 = { static_cast<int>(std::lround(top.x - 1.0 * roi.x)),
                static_cast<int>(std::lround(top.y - 1.0 * roi.y)) };
            finder.input.predicted_point2 = { static_cast<int>(std::lround(bottom.x - 1.0 * roi.x)),
                static_cast<int>(std::lround(bottom.y - 1.0 * roi.y)) };
            if (!finder.solve()) continue;

            candidates.back().upper =
                cv::Point2f { static_cast<float>(finder.result.point1.x + roi.x),
                    static_cast<float>(finder.result.point1.y + roi.y) };
            candidates.back().lower =
                cv::Point2f { static_cast<float>(finder.result.point2.x + roi.x),
                    static_cast<float>(finder.result.point2.y + roi.y) };
            candidates.back().detected = true;
        }

        if (candidates.empty()) return std::nullopt;

        const auto detected = std::ranges::find_if(
            candidates, [](const CandidateRoi& candidate) { return candidate.detected; });
        if (detected == candidates.end()) {
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

        for (std::size_t i = 0; i < candidates.size(); ++i) {
            const auto& candidate = candidates[i];
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
