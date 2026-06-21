#include "reprojection.hpp"

#include <opencv2/calib3d.hpp>

namespace rmcs::details {

auto project_points::impl(std::span<const cv::Point3f> object_points, const cv::Mat& intrinsic,
    const cv::Mat& distortion, std::span<cv::Point2f> projected_points) -> bool {
    auto projected = std::vector<cv::Point2f> { };
    cv::projectPoints(std::vector<cv::Point3f> { object_points.begin(), object_points.end() },
        cv::Vec3d { 0.0, 0.0, 0.0 }, cv::Vec3d { 0.0, 0.0, 0.0 }, intrinsic, distortion, projected);

    if (projected.size() != object_points.size()) return false;

    std::ranges::copy(projected, projected_points.begin());
    return true;
}

}

namespace rmcs::util {

auto reproject_point(const Point3d& point_camera, const util::CameraFeature& camera)
    -> std::optional<Point2d> {

    auto projected = std::vector<cv::Point2f> { };
    {
        const auto object_points = std::vector {
            cv::Point3f {
                static_cast<float>(point_camera.x),
                static_cast<float>(point_camera.y),
                static_cast<float>(point_camera.z),
            },
        };
        const auto zero = cv::Vec3d { 0, 0, 0 };
        cv::projectPoints(
            object_points, zero, zero, camera.intrinsic(), camera.distortion(), projected);
    }

    if (projected.empty()) {
        return std::nullopt;
    }
    return Point2d { projected[0] };
}

}
