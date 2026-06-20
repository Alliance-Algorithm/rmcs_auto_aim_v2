#pragma once

#include "utility/math/camera.hpp"

#include <array>
#include <limits>
#include <ranges>
#include <span>

#include <opencv2/core/types.hpp>

namespace rmcs {

namespace details {

    struct project_points {
        bool success = false;
        explicit project_points(std::span<const cv::Point3f> object_points,
            const cv::Mat& intrinsic, const cv::Mat& distortion,
            std::span<cv::Point2f> projected_points)
            : success { impl(object_points, intrinsic, distortion, projected_points) } { };
        explicit operator bool() const { return success; }

    private:
        static auto impl(std::span<const cv::Point3f> object_points, const cv::Mat& intrinsic,
            const cv::Mat& distortion, std::span<cv::Point2f> projected_points) -> bool;
    };

}

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

        if (!details::project_points { input.object_points, input.camera.intrinsic(),
                input.camera.distortion(), result.projected_points })
            return false;

        result.error = 0.0;
        for (const auto& [projected, detected] :
            std::views::zip(result.projected_points, input.image_points)) {
            result.error += cv::norm(projected - detected);
        }

        result.mean_error = result.error / static_cast<double>(N);
        return true;
    }
};

auto reproject_point(const Point3d& point_camera, const util::CameraFeature& camera)
    -> std::optional<Point2d>;

}
