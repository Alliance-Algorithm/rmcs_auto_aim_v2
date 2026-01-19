#include <cmath>
#include <expected>
#include <filesystem>
#include <format>
#include <opencv2/calib3d.hpp>   // 标定核心算法：calibrateCamera, projectPoints
#include <opencv2/core.hpp>      // 核心数据结构：cv::Mat, cv::Size, cv::Point
#include <opencv2/imgcodecs.hpp> // 图像 IO：imread
#include <opencv2/imgproc.hpp>   // 图像处理：cornerSubPix, cvtColor
#include <print>
#include <ranges>
#include <string>
#include <vector>

namespace fs = std::filesystem;

// --- 1. 配置管理 ---
namespace config {
struct CalibrationOptions {
    cv::Size pattern_size { 9, 6 };
    float square_size_m { 0.02f };

    cv::Size subpix_win { 11, 11 };
    cv::Size subpix_zero { 1, 1 };

    cv::TermCriteria subpix_criteria { cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 40,
        0.001 };

    int calib_flags { cv::CALIB_FIX_K3 | cv::CALIB_FIX_ASPECT_RATIO };
};
}

// --- 2. 格式化输出---
template <>
struct std::formatter<cv::Mat> : std::formatter<std::string> {
    auto format(const cv::Mat& mat, format_context& ctx) const {
        std::string result = "[\n";
        for (int i = 0; i < mat.rows; ++i) {
            result += "  ";
            for (int j = 0; j < mat.cols; ++j) {
                double val = (mat.type() == CV_64F) ? mat.at<double>(i, j) : mat.at<float>(i, j);
                result += std::format("{:>10.4f}{}", val, (j == mat.cols - 1 ? "" : ", "));
            }
            result += "\n";
        }
        result += "]";
        return std::formatter<std::string>::format(result, ctx);
    }
};

// --- 3. 业务逻辑实现 ---

auto get_object_points(const config::CalibrationOptions& opts) -> std::vector<cv::Point3f> {
    auto points = std::vector<cv::Point3f> {};
    points.reserve(opts.pattern_size.area());

    for (auto const [y, x] :
        std::views::cartesian_product(std::views::iota(0, opts.pattern_size.height),
            std::views::iota(0, opts.pattern_size.width))) {
        points.emplace_back(static_cast<float>(x) * opts.square_size_m,
            static_cast<float>(y) * opts.square_size_m, 0.0f);
    }
    return points;
}

auto extract_corners(const fs::path& path, const config::CalibrationOptions& opts)
    -> std::expected<std::vector<cv::Point2f>, std::string> {

    auto img = cv::imread(path.string(), cv::IMREAD_GRAYSCALE);
    if (img.empty()) return std::unexpected("IO Error: Read empty");

    auto corners = std::vector<cv::Point2f> {};

    if (!cv::findChessboardCorners(img, opts.pattern_size, corners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE)) {
        return std::unexpected("Pattern not found");
    }

    cv::cornerSubPix(img, corners, opts.subpix_win, opts.subpix_zero, opts.subpix_criteria);
    return corners;
}

auto main() -> int {
    const config::CalibrationOptions opts;
    const auto img_dir = fs::path { "please/place/your/image/directory" };

    auto obj_points  = std::vector<std::vector<cv::Point3f>> {};
    auto img_points  = std::vector<std::vector<cv::Point2f>> {};
    auto actual_size = cv::Size {};

    if (!fs::exists(img_dir)) {
        std::print(stderr, "Error: Image directory does not exist: {}\n", img_dir.string());
        return -1;
    }

    if (fs::is_empty(img_dir)) {
        std::print(stderr, "Error: Image directory is empty: {}\n", img_dir.string());
        return -1;
    }

    const auto obj_template = get_object_points(opts);

    // --- 数据采集阶段 ---
    {
        for (const auto& entry : fs::directory_iterator(img_dir)) {
            if (!entry.is_regular_file()) continue;

            auto result = extract_corners(entry.path(), opts);
            if (result) {
                img_points.emplace_back(std::move(*result));
                obj_points.emplace_back(obj_template);

                if (actual_size.empty()) {
                    auto tmp    = cv::imread(entry.path().string());
                    actual_size = tmp.size();
                }
            }
            std::print("Processing {:<20} -> {}\n", entry.path().filename().string(),
                result ? "OK" : result.error());
        }
    }

    if (img_points.empty()) {
        std::print("Warning:No valid points detected. Abort.\n");
        return -1;
    }

    // --- 相机标定核心 ---
    cv::Mat K, D;
    std::vector<cv::Mat> rvecs, tvecs;
    {
        auto rms = cv::calibrateCamera(
            obj_points, img_points, actual_size, K, D, rvecs, tvecs, opts.calib_flags);

        std::print("\n--- Calibration Success ---\nRMS Error: {:.6f}\n", rms);
    }

    // --- 精度验证 ---
    {
        double total_err_sq = 0.0;
        size_t total_pts    = 0;

        for (auto i : std::views::iota(0uz, obj_points.size())) {
            std::vector<cv::Point2f> projected;
            cv::projectPoints(obj_points[i], rvecs[i], tvecs[i], K, D, projected);

            auto err = cv::norm(img_points[i], projected, cv::NORM_L2);
            total_err_sq += (err * err);
            total_pts += img_points[i].size();
        }

        const auto reproj_rms = std::sqrt(total_err_sq / static_cast<double>(total_pts));
        std::print("Manual Re-proj RMS: {:.6f} px\n", reproj_rms);
    }

    std::print("\nCamera Matrix K:\n{}\n", K);
    std::print("\nDistortion Coefficients D:\n{}\n", D);

    return 0;
}
