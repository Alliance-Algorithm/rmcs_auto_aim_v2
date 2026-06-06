#pragma once

#include "utility/math/camera.hpp"

#include <array>
#include <limits>
#include <ranges>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/types.hpp>

namespace rmcs {

template <std::size_t N>
struct ReprojectionSolution {
    static_assert(N > 0, "ReprojectionSolution requires at least one point");

    struct Input {
        util::CameraFeature camera;
        std::array<cv::Point3f, N> object_points;
        std::array<cv::Point2f, N> image_points;
    } input;

    struct Result {
        std::array<cv::Point2f, N> projected_points;
        double error;
        double mean_error;
    } result;

    auto solve() -> bool {
        result.error      = std::numeric_limits<double>::max();
        result.mean_error = std::numeric_limits<double>::max();

        const auto object_points =
            std::vector<cv::Point3f> { input.object_points.begin(), input.object_points.end() };

        auto projected_points = std::vector<cv::Point2f> { };
        cv::projectPoints(object_points, cv::Vec3d { 0.0, 0.0, 0.0 }, cv::Vec3d { 0.0, 0.0, 0.0 },
            input.camera.intrinsic(), input.camera.distortion(), projected_points);

        if (projected_points.size() != N) return false;

        std::ranges::copy(projected_points, result.projected_points.begin());

        result.error = 0.0;
        for (const auto& [projected, detected] :
            std::views::zip(result.projected_points, input.image_points)) {
            result.error += cv::norm(projected - detected);
        }

        result.mean_error = result.error / static_cast<double>(N);
        return true;
    }
};

}
