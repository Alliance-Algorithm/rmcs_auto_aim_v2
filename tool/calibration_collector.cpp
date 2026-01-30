#include <hikcamera/capturer.hpp>

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <bitset>
#include <chrono>
#include <concepts>
#include <expected>
#include <filesystem>
#include <format>
#include <print>
#include <ranges>
#include <string>
#include <utility>
#include <vector>

namespace rmcs::tool {

template <class T>
concept CameraLike = requires(T cam, hikcamera::Config cfg) {
    cam.configure(cfg);
    { cam.connect() } -> std::same_as<std::expected<void, std::string>>;
    { cam.disconnect() } -> std::same_as<std::expected<void, std::string>>;
    { cam.connected() } -> std::same_as<bool>;
    { cam.read_image() } -> std::same_as<std::expected<cv::Mat, std::string>>;
};

struct CalibrationCollectorConfig {
    cv::Size chessboard_pattern_size { 11, 8 }; // inner corners (cols, rows)
    int max_corners_refine_iterations { 30 };
    double refine_epsilon { 0.01 };
    std::string window_name { "Calibration Collector" };
    std::filesystem::path output_directory { "please put you data" };
};

struct FrameQualityThresholds {
    double brightness_min { 60.0 };
    double brightness_max { 210.0 };
    double contrast_min { 18.0 };
    double laplacian_variance_min { 120.0 };
    double laplacian_variance_noise_high { 50'000.0 };
    double coverage_min_ratio { 0.05 };    // ignored if negative
    double reprojection_sigma_max { 0.5 }; // pixels
};

enum class FrameQualityCode : int {
    Ok = 0,
    TooDark,
    TooBright,
    LowContrast,
    TooBlur,
    InsufficientCoverage,
    UnstableNoise,
    HomographyError,
};

struct FrameQualityFeedback {
    FrameQualityCode code { FrameQualityCode::Ok };
    const char* message { "Frame quality is good" };
};

using namespace std::chrono_literals;

// Forward declarations for clarity
class CalibrationVisualizer;

// =================================================================================
// 1. State Management: Encapsulate all session data into a single struct
// =================================================================================
struct SessionState {
    // Input data for the current frame
    cv::Mat grayscale_frame;
    std::vector<cv::Point2f> chessboard_corners;
    bool chessboard_found { false };
    double chessboard_coverage_ratio { -1.0 };

    // Coverage masks
    std::bitset<9> coverage_mask {};      // Historical coverage (for saved frames)
    std::bitset<9> current_frame_mask {}; // Coverage in the current frame

    // Feedback message state
    int saved_image_count { 0 };
    std::string transient_feedback_message;
    FrameQualityCode last_feedback_code { FrameQualityCode::Ok };
    std::chrono::steady_clock::time_point feedback_message_expiry {};
};

// =================================================================================
// 2. Quality Evaluation: Stateless utility functions
// =================================================================================
namespace CalibrationQualityUtils {

    [[nodiscard]] static auto EvaluateFrameQuality(const cv::Mat& grayscale_frame,
        double coverage_ratio, const FrameQualityThresholds& thresholds)
        -> std::expected<void, FrameQualityFeedback> {
        auto mean_accum   = cv::Scalar {};
        auto stddev_accum = cv::Scalar {};
        cv::meanStdDev(grayscale_frame, mean_accum, stddev_accum);

        const auto brightness = double { mean_accum[0] };
        if (brightness < thresholds.brightness_min) {
            return std::unexpected(FrameQualityFeedback { .code = FrameQualityCode::TooDark,
                .message = "Increase exposure time or lighting" });
        }
        if (brightness > thresholds.brightness_max) {
            return std::unexpected(FrameQualityFeedback {
                .code = FrameQualityCode::TooBright, .message = "Reduce exposure time or gain" });
        }

        const auto contrast = double { stddev_accum[0] };
        if (contrast < thresholds.contrast_min) {
            return std::unexpected(FrameQualityFeedback { .code = FrameQualityCode::LowContrast,
                .message = "Improve contrast: sharper edges or better lighting" });
        }

        auto laplacian_buffer = cv::Mat {};
        cv::Laplacian(grayscale_frame, laplacian_buffer, CV_64F);
        auto lap_mean_accum = cv::Scalar {};
        auto lap_std_accum  = cv::Scalar {};
        cv::meanStdDev(laplacian_buffer, lap_mean_accum, lap_std_accum);
        const auto laplacian_variance = double { lap_std_accum[0] * lap_std_accum[0] };
        if (laplacian_variance < thresholds.laplacian_variance_min) {
            return std::unexpected(FrameQualityFeedback { .code = FrameQualityCode::TooBlur,
                .message = "Refocus the camera (frame too blurry)" });
        }
        if (laplacian_variance > thresholds.laplacian_variance_noise_high
            && contrast < thresholds.contrast_min * 1.2) {
            return std::unexpected(FrameQualityFeedback { .code = FrameQualityCode::UnstableNoise,
                .message = "Noise high; prefer increasing exposure time over gain" });
        }

        if (coverage_ratio >= 0.0 && coverage_ratio < thresholds.coverage_min_ratio) {
            return std::unexpected(
                FrameQualityFeedback { .code = FrameQualityCode::InsufficientCoverage,
                    .message                 = "Move the chessboard closer or increase coverage" });
        }

        return {};
    }

    [[nodiscard]] static auto EvaluateHomographyError(const std::vector<cv::Point2f>& corners,
        const cv::Size& pattern_size, const FrameQualityThresholds& thresholds)
        -> std::expected<void, FrameQualityFeedback> {
        if (corners.size() < 4) {
            return std::unexpected(FrameQualityFeedback { .code = FrameQualityCode::HomographyError,
                .message = "Not enough corners for homography" });
        }

        auto ideal_points = std::vector<cv::Point2f> {};
        ideal_points.reserve(pattern_size.width * pattern_size.height);
        for (auto y = 0; y < pattern_size.height; ++y) {
            for (auto x = 0; x < pattern_size.width; ++x) {
                ideal_points.emplace_back(static_cast<float>(x), static_cast<float>(y));
            }
        }

        const auto homography = cv::findHomography(corners, ideal_points, cv::RANSAC);
        if (homography.empty()) {
            return std::unexpected(FrameQualityFeedback { .code = FrameQualityCode::HomographyError,
                .message = "Homography failed; adjust board pose" });
        }

        auto projected = std::vector<cv::Point2f> {};
        cv::perspectiveTransform(corners, projected, homography);

        auto mean_error_sq = double { 0.0 };
        for (auto i = std::size_t { 0 }; i < projected.size(); ++i) {
            const auto dx = projected[i].x - ideal_points[i].x;
            const auto dy = projected[i].y - ideal_points[i].y;
            mean_error_sq += static_cast<double>(dx * dx + dy * dy);
        }
        mean_error_sq /= static_cast<double>(projected.size());
        const auto sigma = std::sqrt(mean_error_sq);
        if (sigma > thresholds.reprojection_sigma_max) {
            return std::unexpected(FrameQualityFeedback { .code = FrameQualityCode::HomographyError,
                .message = "Detection accuracy low; check board flatness or lighting" });
        }

        return {};
    }

} // namespace CalibrationQualityUtils

// =================================================================================
// 3. UI Rendering: A dedicated class for all drawing operations
// =================================================================================
class CalibrationVisualizer {
public:
    explicit CalibrationVisualizer(std::string window_name)
        : window_name_(std::move(window_name)) {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
    }

    ~CalibrationVisualizer() { cv::destroyWindow(window_name_); }

    CalibrationVisualizer(const CalibrationVisualizer&)                    = delete;
    auto operator=(const CalibrationVisualizer&) -> CalibrationVisualizer& = delete;

    void display_frame(cv::Mat& frame, const SessionState& state, const cv::Size& pattern_size) {
        if (state.chessboard_found) {
            cv::drawChessboardCorners(
                frame, pattern_size, state.chessboard_corners, state.chessboard_found);
        }
        draw_coverage_grid(frame, state.coverage_mask, state.current_frame_mask);
        draw_user_feedback(frame, state);

        cv::imshow(window_name_, frame);
    }

private:
    std::string window_name_;

    static void draw_coverage_grid(
        cv::Mat& frame, const std::bitset<9>& coverage_mask, const std::bitset<9>& current_mask) {
        const auto w      = frame.cols;
        const auto h      = frame.rows;
        const auto cell_w = w / 3;
        const auto cell_h = h / 3;
        for (auto gy = 0; gy < 3; ++gy) {
            for (auto gx = 0; gx < 3; ++gx) {
                const auto idx          = gy * 3 + gx;
                const auto top_left     = cv::Point { gx * cell_w, gy * cell_h };
                const auto bottom_right = cv::Point { (gx + 1) * cell_w, (gy + 1) * cell_h };

                cv::rectangle(
                    frame, top_left, bottom_right, cv::Scalar { 0, 0, 200 }, 1, cv::LINE_8);
                if (coverage_mask.test(static_cast<std::size_t>(idx))) {
                    cv::rectangle(
                        frame, top_left, bottom_right, cv::Scalar { 0, 200, 0 }, 2, cv::LINE_8);
                }
                if (current_mask.test(static_cast<std::size_t>(idx))) {
                    cv::rectangle(
                        frame, top_left, bottom_right, cv::Scalar { 0, 215, 255 }, 2, cv::LINE_8);
                }
            }
        }
    }

    static void draw_user_feedback(cv::Mat& frame, const SessionState& state) {
        const auto persistent_text = std::format("Saved: {}", state.saved_image_count);
        cv::putText(frame, persistent_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8,
            cv::Scalar { 0, 220, 0 }, 2);

        if (!state.transient_feedback_message.empty()) {
            const auto color = state.last_feedback_code == FrameQualityCode::Ok
                ? cv::Scalar { 0, 220, 0 }
                : cv::Scalar { 0, 0, 220 };
            cv::putText(frame, state.transient_feedback_message, cv::Point(10, 70),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, color, 2);
        }
    }
};

// =================================================================================
// 4. Main Controller: Simplified and composed of the above components
// =================================================================================
class CalibrationCollector {
public:
    explicit CalibrationCollector()                                      = default;
    CalibrationCollector(const CalibrationCollector&)                    = delete;
    auto operator=(const CalibrationCollector&) -> CalibrationCollector& = delete;

    template <CameraLike Cam>
    auto Run(Cam& camera, const CalibrationCollectorConfig& config = {}) -> int {
        if (!prepare_output_directory(config.output_directory)) return 1;
        if (!connect_camera(camera)) return 1;

        auto state         = SessionState {};
        auto visualizer    = CalibrationVisualizer { config.window_name };
        auto thresholds    = FrameQualityThresholds {};
        auto preview_frame = cv::Mat {};

        while (true) {
            // 1. Read Frame
            auto frame_result = camera.read_image();
            if (!frame_result) {
                std::println(stderr, "[collector] read failed: {}", frame_result.error());
                continue;
            }
            frame_result.value().copyTo(preview_frame);

            // 2. Process Data & Update State
            update_state_from_frame(state, preview_frame, config);

            // 3. Handle Input & Business Logic
            const auto key = char { static_cast<char>(cv::waitKey(1)) };
            if (key == 'q' || key == 27) {
                break;
            }
            if (key == 's' || key == 'S') {
                handle_save_request(state, thresholds, config, frame_result.value());
            }

            // 4. Update Timed State
            if (std::chrono::steady_clock::now() >= state.feedback_message_expiry) {
                state.transient_feedback_message.clear();
                state.last_feedback_code = FrameQualityCode::Ok;
            }

            // 5. Delegate all rendering to the visualizer
            visualizer.display_frame(preview_frame, state, config.chessboard_pattern_size);
        }

        if (camera.connected()) {
            std::ignore = camera.disconnect();
        }
        return 0;
    }

private:
    // Helper functions to keep the Run loop clean
    static void update_state_from_frame(
        SessionState& state, const cv::Mat& frame, const CalibrationCollectorConfig& config) {
        cv::cvtColor(frame, state.grayscale_frame, cv::COLOR_BGR2GRAY, 1);
        state.chessboard_corners.clear();
        state.current_frame_mask.reset();

        state.chessboard_found = cv::findChessboardCorners(state.grayscale_frame,
            config.chessboard_pattern_size, state.chessboard_corners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);

        if (state.chessboard_found) {
            const auto bounding_rect = cv::Rect { cv::boundingRect(state.chessboard_corners) };
            state.chessboard_coverage_ratio =
                static_cast<double>(bounding_rect.area()) / (frame.cols * frame.rows);

            const auto cell_w = frame.cols / 3.0;
            const auto cell_h = frame.rows / 3.0;
            for (const auto index : state.chessboard_corners
                    | std::views::transform([cell_w, cell_h](const cv::Point2f& pt) {
                          const auto gx = std::clamp(static_cast<int>(pt.x / cell_w), 0, 2);
                          const auto gy = std::clamp(static_cast<int>(pt.y / cell_h), 0, 2);
                          return gy * 3 + gx;
                      })) {
                state.current_frame_mask.set(static_cast<std::size_t>(index));
            }

            const auto criteria =
                cv::TermCriteria { cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS,
                    config.max_corners_refine_iterations, config.refine_epsilon };
            cv::cornerSubPix(
                state.grayscale_frame, state.chessboard_corners, { 5, 5 }, { -1, -1 }, criteria);
        }
    }

    static void handle_save_request(SessionState& state, const FrameQualityThresholds& thresholds,
        const CalibrationCollectorConfig& config, const cv::Mat& raw_frame) {
        const bool is_board_visible = state.chessboard_found
            && (state.chessboard_coverage_ratio >= thresholds.coverage_min_ratio);

        if (!is_board_visible) {
            state.last_feedback_code         = FrameQualityCode::InsufficientCoverage;
            state.transient_feedback_message = "Error: Chessboard not found or too small.";
        } else {
            auto quality_check = CalibrationQualityUtils::EvaluateFrameQuality(
                state.grayscale_frame, state.chessboard_coverage_ratio, thresholds);
            if (!quality_check) {
                state.last_feedback_code         = quality_check.error().code;
                state.transient_feedback_message = quality_check.error().message;
            } else {
                auto homography_check = CalibrationQualityUtils::EvaluateHomographyError(
                    state.chessboard_corners, config.chessboard_pattern_size, thresholds);
                if (!homography_check) {
                    state.last_feedback_code         = homography_check.error().code;
                    state.transient_feedback_message = homography_check.error().message;
                } else {
                    save_image(state, config.output_directory, raw_frame);
                }
            }
        }
        state.feedback_message_expiry = std::chrono::steady_clock::now() + 2s;
    }

    static void save_image(
        SessionState& state, const std::filesystem::path& out_dir, const cv::Mat& raw_frame) {
        const auto filename = MakeTimestampedName(out_dir);
        if (cv::imwrite(filename.string(), raw_frame)) {
            state.saved_image_count++;
            state.coverage_mask |= state.current_frame_mask;
            state.last_feedback_code         = FrameQualityCode::Ok;
            state.transient_feedback_message = std::format("Saved: {}", filename.stem().string());
        } else {
            state.last_feedback_code         = FrameQualityCode::HomographyError;
            state.transient_feedback_message = "Error: Failed to write file!";
        }
    }

    [[nodiscard]] static auto prepare_output_directory(const std::filesystem::path& path) -> bool {
        auto make_dir_error = std::error_code {};
        std::filesystem::create_directories(path, make_dir_error);
        if (make_dir_error) {
            std::println(
                stderr, "[collector] failed to prepare output dir: {}", make_dir_error.message());
            return false;
        }
        return true;
    }

    template <CameraLike Cam>
    [[nodiscard]] static auto connect_camera(Cam& camera) -> bool {
        auto connect_result = camera.connect();
        if (!connect_result) {
            std::println(stderr, "[collector] connect failed: {}", connect_result.error());
            return false;
        }
        return true;
    }

    static auto MakeTimestampedName(const std::filesystem::path& directory_path)
        -> std::filesystem::path {
        const auto now =
            std::chrono::time_point<std::chrono::system_clock> { std::chrono::system_clock::now() };
        const auto zoned_now = std::chrono::zoned_time { std::chrono::current_zone(), now };
        const auto local_time_ms =
            std::chrono::time_point<std::chrono::local_t, std::chrono::milliseconds> {
                std::chrono::floor<std::chrono::milliseconds>(zoned_now.get_local_time())
            };
        const auto milliseconds_part =
            std::chrono::duration_cast<std::chrono::milliseconds>(local_time_ms.time_since_epoch())
            % 1000;
        const auto filename = std::string { std::format("calib_{:%Y%m%d_%H%M%S}_{:03d}.png",
            local_time_ms, static_cast<int>(milliseconds_part.count())) };
        return directory_path / filename;
    }
};

} // namespace rmcs::tool

auto main() -> int {
    using rmcs::tool::CalibrationCollector;
    using rmcs::tool::CalibrationCollectorConfig;

    auto camera = hikcamera::Camera {};

    camera.configure(hikcamera::Config {
        .timeout_ms      = 2'000,
        .exposure_us     = 10'000,
        .framerate       = 30,
        .gain            = 1.0,
        .invert_image    = false,
        .software_sync   = false,
        .trigger_mode    = false,
        .fixed_framerate = true,
    });
    auto collector = CalibrationCollector {};
    return collector.Run(camera, CalibrationCollectorConfig {});
}
