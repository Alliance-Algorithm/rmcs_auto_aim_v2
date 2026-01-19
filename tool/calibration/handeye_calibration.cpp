#define OPENCV_DISABLE_EIGEN_TENSOR_SUPPORT

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <expected>
#include <filesystem>
#include <fstream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <print>
#include <ranges>
#include <string>
#include <string_view>
#include <vector>
#include <yaml-cpp/yaml.h>
namespace fs = std::filesystem;

/**
 * @class ImuCameraCalibrator
 * @brief 用于求解相机与 IMU 之间的静态外参变换矩阵。
 * * 适用于相机与 IMU 共同安装在云台 Pitch 轴（或任何刚性连接件）上的场景。
 */
class ImuCameraCalibrator {
public:
    struct ChessboardConfig {
        cv::Size board_pattern_size;           // 棋盘格内部角点数 (例如 11x8 棋盘格填 Size(10, 7))
        double square_size_meters;             // 方格边长 (m)
        cv::Matx33d camera_matrix;             // 相机内参
        cv::Mat distortion_coefficients;       // 相机畸变
        cv::Vec3d camera_to_imu_translation_m; // CAD 测得的 Camera->IMU 平移 (m)
    };

    struct CalibrationResult {
        cv::Mat rotation_camera_to_imu;
        cv::Mat translation_camera_to_imu;
        double average_reprojection_error; // 评估精度
    };

    explicit ImuCameraCalibrator(ChessboardConfig config)
        : config_ { std::move(config) } { }

    /**
     * @brief 执行标定核心函数
     */
    auto solve(const std::string_view input_directory)
        -> std::expected<CalibrationResult, std::string> {
        return collect_calibration_data(input_directory)
            .and_then([this](auto&& observations) -> std::expected<CalibrationResult, std::string> {
                auto& [imu_seq, cam_seq] = observations;

                if (imu_seq.rotation_matrices.size() < 12) {
                    return std::unexpected("样本多样性不足：有效帧数需 > 12 "
                                           "帧且具备明显的姿态差异。");
                }

                auto rotation_camera_to_imu         = cv::Mat {};
                auto translation_camera_to_imu_pure = cv::Mat {};
                {
                    // 解算块：A = IMU in World，B = Board in Camera
                    cv::calibrateHandEye(imu_seq.rotation_matrices, imu_seq.translation_vectors,
                        cam_seq.rotation_matrices, cam_seq.translation_vectors,
                        rotation_camera_to_imu, translation_camera_to_imu_pure,
                        cv::CALIB_HAND_EYE_DANIILIDIS);
                }

                // Hybrid 块：使用 CAD 固定平移
                const auto translation_camera_to_imu_hybrid = (cv::Mat_<double>(3, 1)
                        << config_.camera_to_imu_translation_m[0],
                    config_.camera_to_imu_translation_m[1], config_.camera_to_imu_translation_m[2]);

                // 评估块：分别计算纯解算与 Hybrid 的残差
                const double error_pure = calculate_reprojection_error(
                    imu_seq, cam_seq, rotation_camera_to_imu, translation_camera_to_imu_pure);
                const double error_hybrid = calculate_reprojection_error(
                    imu_seq, cam_seq, rotation_camera_to_imu, translation_camera_to_imu_hybrid);

                CalibrationResult pure_result { .rotation_camera_to_imu = rotation_camera_to_imu,
                    .translation_camera_to_imu  = translation_camera_to_imu_pure,
                    .average_reprojection_error = error_pure };

                CalibrationResult hybrid_result { .rotation_camera_to_imu = rotation_camera_to_imu,
                    .translation_camera_to_imu  = translation_camera_to_imu_hybrid,
                    .average_reprojection_error = error_hybrid };

                print_calibration_summary(pure_result, hybrid_result);
                return hybrid_result;
            })
            .and_then([](auto&& result) -> std::expected<CalibrationResult, std::string> {
                return result;
            });
    }

private:
    struct ObservationSequence {
        std::vector<cv::Mat> rotation_matrices;
        std::vector<cv::Mat> translation_vectors;
    };

    ChessboardConfig config_;

    /**
     * @brief 数据采集与预处理：提取棋盘格角点并进行 PnP 位姿估计
     */
    auto collect_calibration_data(const std::string_view directory)
        -> std::expected<std::pair<ObservationSequence, ObservationSequence>, std::string> {

        ObservationSequence imu_sequence, camera_sequence;
        const auto object_points = generate_chessboard_object_points();

        for (int index = 1;; ++index) {
            const auto img_path = std::format("{}/{}.jpg", directory, index);
            const auto q_path   = std::format("{}/{}.txt", directory, index);

            if (!fs::exists(img_path)) break;

            auto color_image    = cv::imread(img_path);
            auto imu_quaternion = load_quaternion(q_path);

            auto gray_image = cv::Mat {};
            cv::cvtColor(color_image, gray_image, cv::COLOR_BGR2GRAY);

            auto corners = std::vector<cv::Point2f> {};
            bool found = cv::findChessboardCorners(gray_image, config_.board_pattern_size, corners,
                cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

            if (found) {
                cv::cornerSubPix(gray_image, corners, cv::Size(11, 11), cv::Size(1, 1),
                    cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.01));

                auto rvec            = cv::Mat {};
                auto tvec            = cv::Mat {};
                auto rotation_camera = cv::Mat {};
                cv::solvePnP(object_points, corners, config_.camera_matrix,
                    config_.distortion_coefficients, rvec, tvec, false, cv::SOLVEPNP_IPPE);
                cv::Rodrigues(rvec, rotation_camera);

                // 3. 存储 IMU 位姿 (A) 与 相机位姿 (B)
                auto rotation_imu_cv = cv::Mat {};
                cv::eigen2cv(imu_quaternion.toRotationMatrix(), rotation_imu_cv);

                imu_sequence.rotation_matrices.push_back(rotation_imu_cv);
                imu_sequence.translation_vectors.push_back(cv::Mat::zeros(3, 1, CV_64F));

                camera_sequence.rotation_matrices.push_back(rotation_camera);
                camera_sequence.translation_vectors.push_back(tvec);

                std::println("Frame {}: 角点提取成功。", index);
            }
        }
        return std::make_pair(imu_sequence, camera_sequence);
    }

    /**
     * @brief 计算手眼标定的重投影残差 (评估一致性)
     */
    static auto calculate_reprojection_error(const ObservationSequence& imu_seq,
        const ObservationSequence& cam_seq, const cv::Mat& rotation_camera_to_imu,
        const cv::Mat& translation_camera_to_imu) -> double {
        double total_error = 0.0;
        int frame_count    = static_cast<int>(imu_seq.rotation_matrices.size());

        // 原理：在理想状态下，Board 在 World 系下的位姿在所有帧中应保持一致
        // T_board_in_world = T_imu_in_world * T_camera_in_imu * T_board_in_camera
        std::vector<Eigen::Vector3d> board_centers_in_world;

        for (int i = 0; i < frame_count; ++i) {
            auto imu_to_world_rotation       = Eigen::Matrix3d {};
            auto camera_to_imu_rotation      = Eigen::Matrix3d {};
            auto board_to_camera_rotation    = Eigen::Matrix3d {};
            auto imu_to_world_translation    = Eigen::Vector3d {};
            auto camera_to_imu_translation   = Eigen::Vector3d {};
            auto board_to_camera_translation = Eigen::Vector3d {};

            cv::cv2eigen(imu_seq.rotation_matrices[i], imu_to_world_rotation);
            cv::cv2eigen(imu_seq.translation_vectors[i], imu_to_world_translation);
            cv::cv2eigen(rotation_camera_to_imu, camera_to_imu_rotation);
            cv::cv2eigen(translation_camera_to_imu, camera_to_imu_translation);
            cv::cv2eigen(cam_seq.rotation_matrices[i], board_to_camera_rotation);
            cv::cv2eigen(cam_seq.translation_vectors[i], board_to_camera_translation);

            // 变换链：Target -> Camera -> IMU -> World
            Eigen::Vector3d board_in_world = imu_to_world_rotation
                    * (camera_to_imu_rotation * board_to_camera_translation
                        + camera_to_imu_translation)
                + imu_to_world_translation;
            board_centers_in_world.push_back(board_in_world);
        }

        // 计算所有计算出的标靶中心点的标准差作为残差参考
        Eigen::Vector3d mean_p = Eigen::Vector3d::Zero();
        for (const auto& p : board_centers_in_world)
            mean_p += p;
        mean_p /= frame_count;

        for (const auto& p : board_centers_in_world) {
            total_error += (p - mean_p).norm();
        }

        return total_error / frame_count;
    }

    auto generate_chessboard_object_points() const -> std::vector<cv::Point3f> {
        std::vector<cv::Point3f> points;
        for (auto row : std::views::iota(0, config_.board_pattern_size.height)) {
            for (auto col : std::views::iota(0, config_.board_pattern_size.width)) {
                points.emplace_back(static_cast<float>(col * config_.square_size_meters),
                    static_cast<float>(row * config_.square_size_meters), 0.0f);
            }
        }
        return points;
    }

    static auto load_quaternion(const std::string& path) -> Eigen::Quaterniond {
        std::ifstream fs { path };
        double w, x, y, z;
        return (fs >> w >> x >> y >> z) ? Eigen::Quaterniond { w, x, y, z }
                                        : Eigen::Quaterniond::Identity();
    }

    static auto print_calibration_summary(
        const CalibrationResult& pure, const CalibrationResult& hybrid) -> void {
        auto as_vec3 = [](const cv::Mat& t) -> cv::Vec3d {
            return cv::Vec3d { t.at<double>(0), t.at<double>(1), t.at<double>(2) };
        };

        const auto t_pure   = as_vec3(pure.translation_camera_to_imu);
        const auto t_hybrid = as_vec3(hybrid.translation_camera_to_imu);
        const auto error_delta =
            hybrid.average_reprojection_error - pure.average_reprojection_error;

        std::println("{:=^72}", " Hand-Eye Calibration Summary ");
        std::println("{:<10} {:>12} {:>12} {:>12} {:>12} {:>12}", "Mode", "tx(m)", "ty(m)", "tz(m)",
            "Error", "Delta");
        std::println("{:<10} {:>12.3f} {:>12.3f} {:>12.3f} {:>12.6f} {:>12.6f}", "Pure", t_pure[0],
            t_pure[1], t_pure[2], pure.average_reprojection_error, error_delta * -1.0);
        std::println("{:<10} {:>12.3f} {:>12.3f} {:>12.3f} {:>12.6f} {:>12.6f}", "Hybrid",
            t_hybrid[0], t_hybrid[1], t_hybrid[2], hybrid.average_reprojection_error, error_delta);
        std::println("{:=^72}", "");
    }
};

auto main() -> int {
    const auto data_directory = fs::path { "/path/to/your/handeye/data" };

    if (!fs::exists(data_directory)) {
        std::println(stderr, "Error: data directory not found: {}", data_directory.string());
        return 1;
    }

    auto config = ImuCameraCalibrator::ChessboardConfig {
        .board_pattern_size      = cv::Size { 9, 6 },
        .square_size_meters      = 0.02,                                      // 2 cm
        .camera_matrix           = cv::Matx33d::eye(),                        // TODO: 填入实际内参
        .distortion_coefficients = (cv::Mat_<double>(1, 5) << 0, 0, 0, 0, 0), // TODO: 填入实际畸变
        .camera_to_imu_translation_m = cv::Vec3d { 0.0, 0.0, 0.0 },           // TODO: CAD 实测值
    };

    auto calibrator = ImuCameraCalibrator { std::move(config) };
    auto result     = calibrator.solve(data_directory.string());

    if (!result) {
        std::println(stderr, "Calibration failed: {}", result.error());
        return 1;
    }

    return 0;
}
