/// @brief 灯条角点优化算法
/// @note  算法来源: CSU-FYT-Vision/FYT2024_vision (Apache-2.0)
///        原实现: rm_auto_aim/armor_detector/src/light_corner_corrector.cpp
///
///  算法流程:
///  1. 取灯条 upper/lower 外包矩形 + padding 作为局部 ROI
///  2. ROI 内提取 B-R 或 R-B 通道差分图
///  3. PCA 求灯条对称轴 (centroid + direction)
///  4. 沿对称轴在 [0.3L, 0.7L] 区间搜索亮度梯度最大的点作为角点

#include "corners_optimizor.hpp"
#include "utility/image/image.details.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <optional>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace rmcs::util {

namespace {
    constexpr auto kMaxBrightness = 25.0;
    constexpr auto kAxisStart     = 0.3;
    constexpr auto kAxisEnd       = 0.7;
    constexpr auto kPad           = 5.0;

    struct SymmetryAxis {
        cv::Point2d centroid;
        cv::Point2d direction;
        double mean_value;
    };

    auto in_image(const cv::Mat& image, const cv::Point2d& point) -> bool {
        return point.x >= 0.0 && point.x < image.cols && point.y >= 0.0 && point.y < image.rows;
    }

    auto find_symmetry_axis(const cv::Mat& channel, const cv::Point2d& offset)
        -> std::optional<SymmetryAxis> {

        auto roi = cv::Mat { };
        channel.convertTo(roi, CV_64F);
        cv::normalize(roi, roi, 0.0, kMaxBrightness, cv::NORM_MINMAX);

        const auto moments = cv::moments(roi, false);
        if (std::abs(moments.m00) < 1e-6) return std::nullopt;

        auto points = std::vector<cv::Point2d> { };
        points.reserve(roi.rows * roi.cols);
        for (auto y = 0; y < roi.rows; ++y) {
            for (auto x = 0; x < roi.cols; ++x) {
                const auto repeat = static_cast<int>(std::round(roi.at<double>(y, x)));
                for (auto count = 0; count < repeat; ++count) {
                    points.emplace_back(static_cast<double>(x), static_cast<double>(y));
                }
            }
        }
        if (points.size() < 2) return std::nullopt;

        const auto points_mat = cv::Mat { points }.reshape(1);
        const auto pca        = cv::PCA { points_mat, cv::Mat { }, cv::PCA::DATA_AS_ROW };

        auto direction =
            cv::Point2d { pca.eigenvectors.at<double>(0, 0), pca.eigenvectors.at<double>(0, 1) };
        const auto norm = cv::norm(direction);
        if (norm < 1e-6) return std::nullopt;
        direction /= norm;
        if (direction.y > 0.0) direction = -direction;

        const auto centroid =
            cv::Point2d { moments.m10 / moments.m00, moments.m01 / moments.m00 } + offset;

        return SymmetryAxis {
            .centroid   = centroid,
            .direction  = direction,
            .mean_value = cv::mean(channel)[0],
        };
    }

    auto find_corner(const cv::Mat& channel, const cv::Point2d& offset, double lightbar_length,
        const SymmetryAxis& axis, bool upper) -> std::optional<cv::Point2d> {

        const auto centroid_local = axis.centroid - offset;

        const auto operate = upper ? 1.0 : -1.0;
        const auto delta   = axis.direction * operate;
        const auto normal  = cv::Point2d { -axis.direction.y, axis.direction.x };

        auto candidates       = std::vector<cv::Point2d> { };
        const auto max_length = lightbar_length * (kAxisEnd - kAxisStart);

        for (auto i = -1; i <= 1; ++i) {
            auto start = centroid_local + delta * (lightbar_length * kAxisStart)
                + normal * static_cast<double>(i);

            auto prev       = start;
            auto corner     = start;
            auto max_diff   = 0.0;
            auto has_corner = false;

            for (auto current = start + delta; cv::norm(current - start) < max_length;
                current += delta) {
                if (!in_image(channel, prev) || !in_image(channel, current)) break;

                const auto prev_value    = channel.at<uchar>(cv::Point(prev));
                const auto current_value = channel.at<uchar>(cv::Point(current));
                const auto diff =
                    static_cast<double>(prev_value) - static_cast<double>(current_value);
                if (diff > max_diff && static_cast<double>(prev_value) > axis.mean_value) {
                    max_diff   = diff;
                    corner     = prev;
                    has_corner = true;
                }
                prev = current;
            }

            if (has_corner) candidates.push_back(corner + offset);
        }

        if (candidates.empty()) return std::nullopt;

        const auto sum =
            std::accumulate(candidates.begin(), candidates.end(), cv::Point2d { 0.0, 0.0 });
        return sum * (1.0 / static_cast<double>(candidates.size()));
    }

}

/// @FIXME: 角点优化的实现存在问题，等待后续修复再开启

auto optimize_corners(const Image& image, Lightbar2d& lightbar) -> void {
    return; // @FIXME:

    const auto& mat = image.details().mat;

    if (mat.empty()) return;
    if (mat.channels() < 3) return;
    if (lightbar.color != ArmorColor::BLUE && lightbar.color != ArmorColor::RED) return;

    // 从 upper/lower 计算外包矩形作为 ROI
    const auto ux = static_cast<int>(lightbar.upper.x);
    const auto uy = static_cast<int>(lightbar.upper.y);
    const auto lx = static_cast<int>(lightbar.lower.x);
    const auto ly = static_cast<int>(lightbar.lower.y);

    const auto roi = cv::Rect {
        std::min(ux, lx) - static_cast<int>(kPad),
        std::min(uy, ly) - static_cast<int>(kPad),
        std::abs(lx - ux) + static_cast<int>(kPad * 2) + 1,
        std::abs(ly - uy) + static_cast<int>(kPad * 2) + 1,
    } & cv::Rect { 0, 0, mat.cols, mat.rows };
    if (roi.width <= 1 || roi.height <= 1) return;

    // 提取通道差分图
    auto channel = cv::Mat { };
    {
        const auto image_roi = mat(roi);
        auto b_channel       = cv::Mat { };
        auto r_channel       = cv::Mat { };
        cv::extractChannel(image_roi, b_channel, 0);
        cv::extractChannel(image_roi, r_channel, 2);

        if (lightbar.color == ArmorColor::BLUE) {
            cv::subtract(b_channel, r_channel, channel);
        } else {
            cv::subtract(r_channel, b_channel, channel);
        }
    }

    // PCA 求对称轴
    const auto offset = cv::Point2d {
        static_cast<double>(roi.x),
        static_cast<double>(roi.y),
    };
    const auto axis = find_symmetry_axis(channel, offset);
    if (!axis) return;

    // 沿对称轴搜索端点
    const auto dx     = lightbar.upper.x - lightbar.lower.x;
    const auto dy     = lightbar.upper.y - lightbar.lower.y;
    const auto length = std::hypot(dx, dy);

    if (auto upper = find_corner(channel, offset, length, *axis, true)) {
        lightbar.upper = Point2d { upper->x, upper->y };
    }
    if (auto lower = find_corner(channel, offset, length, *axis, false)) {
        lightbar.lower = Point2d { lower->x, lower->y };
    }
}

auto optimize_corners(const Image& image, Armor2ds& armors) -> void {
    return; // @FIXME:

    for (auto& armor : armors) {
        auto left = Lightbar2d {
            .color = armor.color,
            .upper = Point2d { static_cast<double>(armor.tl.x), static_cast<double>(armor.tl.y) },
            .lower = Point2d { static_cast<double>(armor.bl.x), static_cast<double>(armor.bl.y) },
        };
        auto right = Lightbar2d {
            .color = armor.color,
            .upper = Point2d { static_cast<double>(armor.tr.x), static_cast<double>(armor.tr.y) },
            .lower = Point2d { static_cast<double>(armor.br.x), static_cast<double>(armor.br.y) },
        };

        optimize_corners(image, left);
        optimize_corners(image, right);

        armor.tl     = { static_cast<float>(left.upper.x), static_cast<float>(left.upper.y) };
        armor.bl     = { static_cast<float>(left.lower.x), static_cast<float>(left.lower.y) };
        armor.tr     = { static_cast<float>(right.upper.x), static_cast<float>(right.upper.y) };
        armor.br     = { static_cast<float>(right.lower.x), static_cast<float>(right.lower.y) };
        armor.center = (armor.tl + armor.tr + armor.br + armor.bl) * 0.25f;
    }
}

}
