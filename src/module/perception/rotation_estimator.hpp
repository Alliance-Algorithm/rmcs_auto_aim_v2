#pragma once
#include "utility/clock.hpp"
#include "utility/image/image.hpp"
#include "utility/pimpl.hpp"
#include "utility/robot/armor.hpp"

#include <array>
#include <optional>
#include <span>
#include <vector>

namespace rmcs {

// 这是一段示例代码，用于发布估计的角加速度
//
// if (auto result = rotation_estimator.update(*image)) {
//     const auto& addition = rotation_estimator.addition();
//     for (const auto& point : addition.tracked_features) {
//         visual.draw_later(Canvas::Point {
//             .origin = point.make<cv::Point2i>(),
//             .radius = 1,
//             .color  = kGreen,
//         });
//     }
//     visual.publish(-result->yaw_rate, "estimate_yaw");
//     visual.publish(-result->pitch_rate, "estimate_pitch");
// }

class RotationEstimator {
    RMCS_PIMPL_DEFINITION(RotationEstimator)

public:
    struct Estimate {
        TimePoint timestamp;
        double dt;

        double yaw_rate;
        double pitch_rate;
        double roll_rate;

        double confidence;
    };
    struct Addition {
        TimePoint timestamp { };
        double dt { 0.0 };

        bool valid { false };

        int tracked_points { 0 };
        int inlier_points { 0 };
        int feature_points { 0 };

        double scale { 1.0 };
        double confidence { 0.0 };

        std::vector<Point2d> tracked_features;
    };

    auto configure(std::array<double, 9> camera_matrix) -> void;
    auto configure(std::array<double, 5> distort_coeff) -> void;

    auto update(const Image& image) -> std::optional<Estimate>;

    auto update_mask(std::span<const Armor2d> armors) -> void;
    auto update_mask(std::span<const Lightbar2d> lightbars) -> void;

    auto addition() const -> const Addition&;

    auto reset() -> void;
};

}
