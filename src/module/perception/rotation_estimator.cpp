/// @brief 基于成熟 VIO 视觉前端思想的轻量旋转速度估计器。
///
/// 算法来源：
/// - VINS-Mono FeatureTracker:
///   https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/master/feature_tracker/src/feature_tracker.cpp
/// - OpenVINS TrackKLT:
///   https://github.com/rpng/open_vins/blob/master/ov_core/src/track/TrackKLT.cpp
///
/// 本实现没有复制上述 GPL 项目源码，只参考其通用流程：维护上一帧特征点，使用 KLT
/// 光流跟踪到当前帧，通过 mask 排除装甲板等动态区域，使用 RANSAC 剔除外点，再根据
/// 归一化图像平面上的背景点速度输出 yaw/pitch/roll-like 视觉旋转速度波形。

#include "rotation_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

using namespace rmcs;

struct RotationEstimator::Impl {
    static constexpr auto kMaxFeatures       = 160;
    static constexpr auto kMinTrackedPoints  = 6;
    static constexpr auto kMinInlierPoints   = 4;
    static constexpr auto kRefillThreshold   = 80;
    static constexpr auto kMinFeatureDist    = 18.0;
    static constexpr auto kMaskPaddingRatio  = 0.65;
    static constexpr auto kScale             = 0.3;
    static constexpr auto kMinDt             = 1e-4;
    static constexpr auto kMaxDt             = 0.2;
    static constexpr auto kBorderSize        = 8;
    static constexpr auto kMaxOpticalFlowErr = 80.0f;
    static constexpr auto kGridStep          = 48;

    struct Tracks {
        std::vector<cv::Point2f> prev;
        std::vector<cv::Point2f> current;
        std::vector<cv::Point2f> prev_inliers;
        std::vector<cv::Point2f> current_inliers;
        double roll_rate_like { 0.0 };
    };

    cv::Mat prev_gray;
    cv::Mat armor_mask;
    cv::Mat lightbar_mask;
    TimePoint prev_timestamp { };
    std::vector<cv::Point2f> prev_points;

    cv::Mat camera_matrix;
    cv::Mat distort_coeff;
    Addition addition { };

    static auto in_border(const cv::Point2f& point, const cv::Size& size) -> bool {
        const auto border = static_cast<float>(kBorderSize);
        const auto width  = static_cast<float>(size.width - kBorderSize);
        const auto height = static_cast<float>(size.height - kBorderSize);

        return point.x >= border && point.y >= border && point.x < width && point.y < height;
    }

    static auto mask_allows(const cv::Mat& mask, const cv::Point2f& point) -> bool {
        const auto x = static_cast<int>(std::lround(point.x));
        const auto y = static_cast<int>(std::lround(point.y));
        if (x < 0 || y < 0 || x >= mask.cols || y >= mask.rows) return false;

        return mask.at<std::uint8_t>(y, x) != 0;
    }

    static auto median(std::vector<double> values) -> double {
        if (values.empty()) return 0.0;

        const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
        std::nth_element(values.begin(), middle, values.end());

        auto result = *middle;
        if (values.size() % 2 == 0) {
            const auto lower = std::max_element(values.begin(), middle);
            result           = (*lower + result) * 0.5;
        }
        return result;
    }

    static auto detect_features(const cv::Mat& gray, const cv::Mat& allowed_mask,
        const std::vector<cv::Point2f>& existing) -> std::vector<cv::Point2f> {
        auto feature_mask = allowed_mask.clone();
        auto compacted    = std::vector<cv::Point2f> { };
        compacted.reserve(kMaxFeatures);

        for (const auto& point : existing) {
            if (!in_border(point, gray.size()) || !mask_allows(allowed_mask, point)) continue;

            compacted.push_back(point);
            cv::circle(feature_mask, point, static_cast<int>(kMinFeatureDist), cv::Scalar { 0 },
                cv::FILLED);

            if (static_cast<int>(compacted.size()) >= kMaxFeatures) return compacted;
        }

        {
            auto new_points   = std::vector<cv::Point2f> { };
            const auto needed = kMaxFeatures - static_cast<int>(compacted.size());
            if (needed > 0) {
                cv::goodFeaturesToTrack(
                    gray, new_points, needed, 0.01, kMinFeatureDist, feature_mask, 3, false, 0.04);
            }
            compacted.insert(compacted.end(), new_points.begin(), new_points.end());
        }

        if (static_cast<int>(compacted.size()) >= kMinTrackedPoints) return compacted;

        for (auto y = kGridStep / 2; y < gray.rows; y += kGridStep) {
            for (auto x = kGridStep / 2; x < gray.cols; x += kGridStep) {
                auto point = cv::Point2f { static_cast<float>(x), static_cast<float>(y) };
                if (!mask_allows(allowed_mask, point)) continue;

                compacted.push_back(point);
                if (static_cast<int>(compacted.size()) >= kMaxFeatures) return compacted;
            }
        }
        return compacted;
    }

    static auto into_original_points(const std::vector<cv::Point2f>& points)
        -> std::vector<Point2d> {
        auto result = std::vector<Point2d> { };
        result.reserve(points.size());

        constexpr auto inv_scale = 1.0 / kScale;
        for (const auto& point : points) {
            result.emplace_back(cv::Point2d { point.x * inv_scale, point.y * inv_scale });
        }
        return result;
    }

    auto configure_camera_matrix(std::array<double, 9> input) -> void {
        camera_matrix = cv::Mat(3, 3, CV_64F);
        for (auto i = 0; i < 9; ++i) {
            camera_matrix.at<double>(i / 3, i % 3) = input[static_cast<std::size_t>(i)];
        }
    }

    auto configure_distort_coeff(std::array<double, 5> input) -> void {
        distort_coeff = cv::Mat(1, 5, CV_64F);
        for (auto i = 0; i < 5; ++i) {
            distort_coeff.at<double>(0, i) = input[static_cast<std::size_t>(i)];
        }
    }

    auto reset() -> void {
        prev_gray.release();
        armor_mask.release();
        lightbar_mask.release();
        prev_points.clear();
        prev_timestamp = { };
        addition       = { };
    }

    auto update_mask(std::span<const Armor2d> armors) -> void {
        if (prev_gray.empty()) return;

        auto mask = cv::Mat { prev_gray.size(), CV_8UC1, cv::Scalar { 255 } };
        for (const auto& armor : armors) {
            auto points = armor.points();
            for (auto& p : points) {
                p.x *= static_cast<float>(kScale);
                p.y *= static_cast<float>(kScale);
            }

            auto rect = cv::boundingRect(points);
            {
                const auto pad = static_cast<int>(
                    std::lround(std::max(rect.width, rect.height) * kMaskPaddingRatio));
                rect.x -= pad;
                rect.y -= pad;
                rect.width += pad * 2;
                rect.height += pad * 2;
                rect &= cv::Rect { 0, 0, mask.cols, mask.rows };
            }
            if (rect.area() > 0) cv::rectangle(mask, rect, cv::Scalar { 0 }, cv::FILLED);
        }
        armor_mask = mask;
    }

    auto update_mask(std::span<const Lightbar2d> lightbars) -> void {
        if (prev_gray.empty()) return;

        auto mask = cv::Mat { prev_gray.size(), CV_8UC1, cv::Scalar { 255 } };
        for (const auto& bar : lightbars) {
            const auto ux = static_cast<float>(bar.upper.x * kScale);
            const auto uy = static_cast<float>(bar.upper.y * kScale);
            const auto lx = static_cast<float>(bar.lower.x * kScale);
            const auto ly = static_cast<float>(bar.lower.y * kScale);

            auto points = std::vector<cv::Point2f> {
                cv::Point2f { ux, uy },
                cv::Point2f { lx, ly },
            };

            auto rect = cv::boundingRect(points);
            {
                const auto pad = static_cast<int>(
                    std::lround(std::max(rect.width, rect.height) * kMaskPaddingRatio));
                rect.x -= pad;
                rect.y -= pad;
                rect.width += pad * 2;
                rect.height += pad * 2;
                rect &= cv::Rect { 0, 0, mask.cols, mask.rows };
            }
            if (rect.area() > 0) cv::rectangle(mask, rect, cv::Scalar { 0 }, cv::FILLED);
        }
        lightbar_mask = mask;
    }

    auto update(const Image& image) -> std::optional<Estimate> {

        const auto& input = image.mat;
        if (input.empty()) return std::nullopt;

        auto raw_gray = cv::Mat { };
        if (input.channels() == 1) {
            raw_gray = input.clone();
        } else {
            cv::cvtColor(input, raw_gray, cv::COLOR_BGR2GRAY);
        }

        auto gray = cv::Mat { };
        cv::resize(raw_gray, gray, cv::Size { }, kScale, kScale, cv::INTER_AREA);

        auto allowed_mask = cv::Mat { gray.size(), CV_8UC1, cv::Scalar { 255 } };
        if (!armor_mask.empty()) {
            cv::bitwise_and(allowed_mask, armor_mask, allowed_mask);
        }
        if (!lightbar_mask.empty()) {
            cv::bitwise_and(allowed_mask, lightbar_mask, allowed_mask);
        }

        const auto timestamp = image.timestamp;

        addition           = { };
        addition.timestamp = timestamp;
        addition.scale     = kScale;

        if (prev_gray.empty()) {
            prev_points             = detect_features(gray, allowed_mask, { });
            prev_gray               = gray;
            prev_timestamp          = timestamp;
            addition.feature_points = static_cast<int>(prev_points.size());
            return std::nullopt;
        }

        const auto dt = std::chrono::duration<double>(timestamp - prev_timestamp).count();
        addition.dt   = dt;
        if (!std::isfinite(dt) || dt < kMinDt || dt > kMaxDt || prev_points.empty()) {
            prev_points             = detect_features(gray, allowed_mask, { });
            prev_gray               = gray;
            prev_timestamp          = timestamp;
            addition.feature_points = static_cast<int>(prev_points.size());
            return std::nullopt;
        }

        const auto tracked        = track(prev_gray, gray, allowed_mask);
        const auto tracked_points = static_cast<int>(tracked.prev.size());
        addition.tracked_points   = tracked_points;
        addition.inlier_points    = static_cast<int>(tracked.current_inliers.size());
        addition.tracked_features = into_original_points(tracked.current_inliers);

        const auto estimate = estimate_rotation(tracked, timestamp, dt, tracked_points);
        if (estimate) {
            addition.valid      = true;
            addition.confidence = estimate->confidence;
        }

        const auto next_points = estimate ? tracked.current_inliers : std::vector<cv::Point2f> { };
        if (static_cast<int>(next_points.size()) >= kRefillThreshold) {
            prev_points = next_points;
        } else {
            prev_points = detect_features(gray, allowed_mask, next_points);
        }
        prev_gray               = gray;
        prev_timestamp          = timestamp;
        addition.feature_points = static_cast<int>(prev_points.size());

        return estimate;
    }

    auto track(const cv::Mat& from, const cv::Mat& to, const cv::Mat& mask) const -> Tracks {
        auto result = Tracks { };
        if (prev_points.empty()) return result;

        auto next_points = std::vector<cv::Point2f> { };
        auto status      = std::vector<std::uint8_t> { };
        auto error       = std::vector<float> { };
        cv::calcOpticalFlowPyrLK(from, to, prev_points, next_points, status, error,
            cv::Size { 21, 21 }, 3,
            cv::TermCriteria { cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.01 });

        for (auto i = std::size_t { 0 }; i < next_points.size(); ++i) {
            if (!status[i]) continue;
            if (i < error.size() && error[i] > kMaxOpticalFlowErr) continue;
            if (!in_border(prev_points[i], from.size()) || !in_border(next_points[i], to.size()))
                continue;
            if (!mask_allows(mask, prev_points[i]) || !mask_allows(mask, next_points[i])) continue;

            result.prev.push_back(prev_points[i]);
            result.current.push_back(next_points[i]);
        }

        if (result.prev.size() >= kMinTrackedPoints) {
            auto inliers = std::vector<std::uint8_t> { };
            auto affine  = cv::estimateAffinePartial2D(
                result.prev, result.current, inliers, cv::RANSAC, 3.0, 2'000, 0.99, 10);

            if (!affine.empty()) {
                const auto angle = std::atan2(affine.at<double>(1, 0), affine.at<double>(0, 0));
                result.roll_rate_like = angle;
            }

            for (auto i = std::size_t { 0 }; i < inliers.size(); ++i) {
                if (!inliers[i]) continue;

                result.prev_inliers.push_back(result.prev[i]);
                result.current_inliers.push_back(result.current[i]);
            }

            if (result.prev_inliers.size() < kMinInlierPoints) {
                result.prev_inliers    = result.prev;
                result.current_inliers = result.current;
                result.roll_rate_like  = 0.0;
            }
        }

        return result;
    }

    auto estimate_rotation(const Tracks& tracks, TimePoint timestamp, double dt,
        int tracked_points) const -> std::optional<Estimate> {
        if (tracked_points < kMinTrackedPoints
            || static_cast<int>(tracks.current_inliers.size()) < kMinInlierPoints) {
            return std::nullopt;
        }

        if (camera_matrix.empty() || camera_matrix.rows != 3 || camera_matrix.cols != 3
            || camera_matrix.type() != CV_64F) {
            return std::nullopt;
        }

        const auto fx = camera_matrix.at<double>(0, 0) * kScale;
        const auto fy = camera_matrix.at<double>(1, 1) * kScale;
        const auto cx = camera_matrix.at<double>(0, 2) * kScale;
        const auto cy = camera_matrix.at<double>(1, 2) * kScale;
        if (!std::isfinite(fx) || !std::isfinite(fy) || std::abs(fx) < 1e-6
            || std::abs(fy) < 1e-6) {
            return std::nullopt;
        }

        auto xs = std::vector<double> { };
        auto ys = std::vector<double> { };
        xs.reserve(tracks.current_inliers.size());
        ys.reserve(tracks.current_inliers.size());

        for (auto i = std::size_t { 0 }; i < tracks.current_inliers.size(); ++i) {
            const auto prev_x    = (tracks.prev_inliers[i].x - cx) / fx;
            const auto prev_y    = (tracks.prev_inliers[i].y - cy) / fy;
            const auto current_x = (tracks.current_inliers[i].x - cx) / fx;
            const auto current_y = (tracks.current_inliers[i].y - cy) / fy;

            xs.push_back((current_x - prev_x) / dt);
            ys.push_back((current_y - prev_y) / dt);
        }

        const auto inlier_points = static_cast<int>(tracks.current_inliers.size());
        const auto inlier_ratio =
            tracked_points > 0 ? static_cast<double>(inlier_points) / tracked_points : 0.0;
        const auto count_score = std::clamp(static_cast<double>(inlier_points) / 80.0, 0.0, 1.0);

        return Estimate {
            .timestamp  = timestamp,
            .dt         = dt,
            .yaw_rate   = -median(std::move(xs)),
            .pitch_rate = +median(std::move(ys)),
            .roll_rate  = tracks.roll_rate_like / dt,
            .confidence = std::clamp(count_score * inlier_ratio, 0.0, 1.0),
        };
    }
};

auto RotationEstimator::configure(std::array<double, 9> camera_matrix) -> void {
    pimpl->configure_camera_matrix(camera_matrix);
}

auto RotationEstimator::configure(std::array<double, 5> distort_coeff) -> void {
    pimpl->configure_distort_coeff(distort_coeff);
}

auto RotationEstimator::update(const Image& image) -> std::optional<Estimate> {
    return pimpl->update(image);
}

auto RotationEstimator::update_mask(std::span<const Armor2d> armors) -> void {
    pimpl->update_mask(armors);
}

auto RotationEstimator::update_mask(std::span<const Lightbar2d> lightbars) -> void {
    pimpl->update_mask(lightbars);
}

auto RotationEstimator::reset() -> void { pimpl->reset(); }

auto RotationEstimator::addition() const -> const Addition& { return pimpl->addition; }

RotationEstimator::RotationEstimator() noexcept
    : pimpl { std::make_unique<Impl>() } { }

RotationEstimator::~RotationEstimator() noexcept = default;
