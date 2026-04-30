#define OPENCV_DISABLE_EIGEN_TENSOR_SUPPORT

#include "yaw_optimizer.hpp"
#include "utility/math/angle.hpp"
#include "utility/math/conversion.hpp"

#include <cmath>
#include <limits>
#include <vector>

#include <eigen3/Eigen/Geometry>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

using namespace rmcs::util;

auto compute_reprojection_error(double yaw, double pitch, Eigen::Matrix3d const& r_wc_ros,
    Eigen::Vector3d const& t_wc_ros, Eigen::Vector3d const& xyz_w, cv::Mat const& camera_intrinsic,
    cv::Mat const& camera_distortion, std::vector<cv::Point3f> const& armor_shape_ocv,
    std::vector<cv::Point2f> const& detected_ocv) -> double {
    auto q_aw     = Eigen::Quaterniond { euler_to_quaternion(yaw, pitch, 0.0) };
    auto r_aw_ros = Eigen::Matrix3d { q_aw.toRotationMatrix() };

    auto r_ac_ocv = Eigen::Matrix3d { ros2opencv_rotation(r_wc_ros * r_aw_ros) };
    auto t_ac_ocv = Eigen::Vector3d { ros2opencv_position(r_wc_ros * xyz_w + t_wc_ros) };

    auto r_ac_ocv_cv = cv::Mat { };
    cv::eigen2cv(r_ac_ocv, r_ac_ocv_cv);

    auto rvec = cv::Vec3d { };
    cv::Rodrigues(r_ac_ocv_cv, rvec);
    auto tvec = cv::Vec3d { t_ac_ocv[0], t_ac_ocv[1], t_ac_ocv[2] };

    auto projected = std::vector<cv::Point2f> { };
    cv::projectPoints(armor_shape_ocv, rvec, tvec, camera_intrinsic, camera_distortion, projected);

    auto error = double { 0.0 };
    for (auto j = int { }; j < 4; ++j)
        error += cv::norm(detected_ocv[j] - projected[j]);

    return error;
}

auto YawOptimizer::solve() -> Output {
    constexpr double kSearchRangeDeg { 140.0 };
    constexpr double kSearchStepDeg { 1.0 };
    constexpr double kDefaultPitchDeg { 15.0 };
    constexpr double kOutpostPitchDeg { -15.0 };

    const auto pitch = double { (input.genre == DeviceId::OUTPOST) ? deg2rad(kOutpostPitchDeg)
                                                                   : deg2rad(kDefaultPitchDeg) };

    const auto yaw_start = double { input.center_yaw - deg2rad(kSearchRangeDeg / 2.0) };

    auto camera_intrinsic  = input.camera.intrinsic();
    auto camera_distortion = input.camera.distortion();

    auto q_wc_ros = input.camera.world_to_camera_orientation.make<Eigen::Quaterniond>();
    auto r_wc_ros = q_wc_ros.toRotationMatrix();
    auto t_wc_ros = input.camera.world_to_camera_translation.make<Eigen::Vector3d>();
    auto xyz_w    = input.xyz_in_world.make<Eigen::Vector3d>();

    auto armor_shape_ocv = std::vector<cv::Point3f> { };
    armor_shape_ocv.reserve(4);
    for (const auto& pt : input.armor_shape)
        armor_shape_ocv.emplace_back(pt.x, pt.y, pt.z);

    auto detected_ocv = std::vector<cv::Point2f> { };
    detected_ocv.reserve(4);
    for (const auto& pt : input.detected_corners)
        detected_ocv.emplace_back(pt.x, pt.y);

    auto best_error = double { std::numeric_limits<double>::max() };
    auto best_yaw   = double { input.center_yaw };

    for (auto i = int { }; i < static_cast<int>(kSearchRangeDeg); ++i) {
        auto candidate_yaw = double { yaw_start + i * deg2rad(kSearchStepDeg) };
        auto error = compute_reprojection_error(candidate_yaw, pitch, r_wc_ros, t_wc_ros, xyz_w,
            camera_intrinsic, camera_distortion, armor_shape_ocv, detected_ocv);

        if (error < best_error) {
            best_error = error;
            best_yaw   = candidate_yaw;
        }
    }

    // 三点二次插值，帧间连续稳定
    auto step        = double { deg2rad(kSearchStepDeg) };
    auto e_lo        = compute_reprojection_error(best_yaw - step, pitch, r_wc_ros, t_wc_ros, xyz_w,
        camera_intrinsic, camera_distortion, armor_shape_ocv, detected_ocv);
    auto e_hi        = compute_reprojection_error(best_yaw + step, pitch, r_wc_ros, t_wc_ros, xyz_w,
        camera_intrinsic, camera_distortion, armor_shape_ocv, detected_ocv);
    auto denominator = e_lo - 2.0 * best_error + e_hi + 1e-9;
    auto refined     = best_yaw + step * 0.5 * (e_lo - e_hi) / denominator;

    auto q_aw_best     = euler_to_quaternion(refined, pitch, 0.0);
    auto r_aw_best_ros = q_aw_best.toRotationMatrix();
    auto r_ac_ros      = r_wc_ros * r_aw_best_ros;

    return { Orientation { Eigen::Quaterniond(r_ac_ros).normalized() } };
}
