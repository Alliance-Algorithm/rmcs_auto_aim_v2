#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <experimental/scope>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/board_clock.hpp>
#include <rmcs_msgs/camera_frame.hpp>
#include <rmcs_utility/atomic_futex.hpp>
#include <rmcs_utility/pooled_shared_factory.hpp>
#include <rmcs_utility/ring_buffer.hpp>

#include "hikcamera.hpp"
#include "imu_snapshot_buffer.hpp"
#include "linear_sync_model.hpp"

namespace rmcs {

class AutoAimCapturerComponent final : public rmcs_executor::Component, public rclcpp::Node {
public:
    AutoAimCapturerComponent()
        : Node(get_component_name(),
              rclcpp::NodeOptions { }.automatically_declare_parameters_from_overrides(true))
        , camera_config_ { Hikcamera::CameraConfig::cs016_default()
                .set_device_name(get_parameter_or<std::string>("camera_name", ""))
                .set_exposure_us(
                    static_cast<float>(get_parameter_or<double>("exposure_us", 2000.0)))
                .set_gain(
                    static_cast<float>(get_parameter_or<double>("gain", Hikcamera::Cs016MaxGain)))
                .set_framerate(static_cast<float>(
                    get_parameter_or<double>("framerate", Hikcamera::Cs016MaxFramerate)))
                .set_invert_image(get_parameter_or<bool>("invert_image", false)) }
        , half_exposure_time_(std::chrono::duration_cast<rmcs_msgs::BoardClock::duration>(
              std::chrono::duration<float, std::micro>(camera_config_.exposure_us)))
        , imu_delay_(std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              std::chrono::duration<double, std::milli>(get_parameter_or<double>("delay_ms", 0.0))))
        , matching_array_([] {
            auto arr = std::vector<LinearSyncModel::TimestampPair> { };
            arr.reserve(1000);
            return arr;
        }())
        , sync_model_(
              get_parameter_or<double>("rls_tau_sec", 10.0), (1.0 / camera_config_.framerate) / 2.0)
        , use_hardware_sync_(get_parameter_or<bool>("use_hardware_sync", true)) {
        if (use_hardware_sync_) {
            register_input("/gimbal/auto_aim/exposure_signal", signal_input_);
        }
        register_output("/gimbal/auto_aim/camera_frame", frame_output_);
    }

    ~AutoAimCapturerComponent() override {
        stop_requested_.test_and_set(std::memory_order::relaxed);
        notify_event();

        if (worker_thread_.joinable()) worker_thread_.join();
    }

private:
    void before_updating() override {
        worker_thread_ = std::thread { [this] { worker_main(); } };
    }

    void update() override { }

    void signal_callback(rmcs_msgs::BoardClock::time_point timestamp) {
        const auto now = std::chrono::steady_clock::now();
        if (!unmatched_signal_buffer_.emplace_back(timestamp, now)) {
            RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 1000,
                "Dropping trigger signal: unmatched signal buffer full (capacity=%zu)",
                unmatched_signal_buffer_.max_size());
            return;
        }

        notify_event();
    }

    void frame_callback(const Hikcamera::Frame& frame) {
        if (frame.width != Hikcamera::Cs016FrameWidth || frame.height != Hikcamera::Cs016FrameHeight
            || frame.data.size() != kExpectedFrameSize) {
            RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 1000,
                "Skipping malformed frame #%u: width=%u (expected %u), height=%u (expected %u), "
                "size=%zu (expected %zu)",
                frame.frame_id, frame.width, Hikcamera::Cs016FrameWidth, frame.height,
                Hikcamera::Cs016FrameHeight, frame.data.size(), kExpectedFrameSize);
            return;
        }

        auto opencv_cvt_color_code = int { };
        if (frame.pixel_type == PixelType_Gvsp_BayerRG8) {
            opencv_cvt_color_code = cv::COLOR_BayerRGGB2BGR_EA;
        } else if (frame.pixel_type == PixelType_Gvsp_BayerBG8) {
            opencv_cvt_color_code = cv::COLOR_BayerBGGR2BGR_EA;
        } else if (frame.pixel_type == PixelType_Gvsp_BayerGR8) {
            opencv_cvt_color_code = cv::COLOR_BayerGRBG2BGR_EA;
        } else if (frame.pixel_type == PixelType_Gvsp_BayerGB8) {
            opencv_cvt_color_code = cv::COLOR_BayerGBRG2BGR_EA;
        } else {
            RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 1000,
                "Skipping frame #%u with unsupported pixel type: 0x%lX", frame.frame_id,
                static_cast<long>(frame.pixel_type));
            return;
        }

        last_frame_time_.store(frame.host_timestamp, std::memory_order::release);
        auto output_frame = output_frame_factory_.try_make();
        if (!output_frame) {
            RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 1000,
                "Dropping frame #%u: frame pool exhausted (capacity=%zu)", frame.frame_id,
                output_frame_factory_.max_size());
            return;
        }

        std::memcpy(output_frame->data_raw.data(), frame.data.data(), kExpectedFrameSize);
        output_frame->opencv_cvt_color_code     = opencv_cvt_color_code;
        output_frame->image_reception_timestamp = frame.host_timestamp;

        if (!unmatched_image_buffer_.emplace_back_n(
                [&](std::byte* storage) noexcept {
                    new (storage) UnmatchedImage { output_frame, frame.frame_id, frame.timestamp };
                },
                1)) {
            RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 1000,
                "Dropping frame #%u: unmatched image buffer full (capacity=%zu)", frame.frame_id,
                unmatched_image_buffer_.max_size());
            return;
        }

        notify_event();
    }

    void notify_event() {
        event_count_.fetch_add(1, std::memory_order::release);
        rmcs_utility::atomic_futex_notify_one(event_count_);
    }

    void worker_main() {
        try {
            auto cleanup = std::experimental::scope_exit([this] { camera_.reset(); });
            worker_fsm();
        } catch (const StopRequestException&) {
            return;
        } catch (const std::exception& ex) {
            RCLCPP_FATAL(logger_, "Unhandled exception in worker thread: %s", ex.what());
            throw;
        } catch (...) {
            RCLCPP_FATAL(logger_, "Unhandled non-std exception in worker thread");
            throw;
        }
    }

    void worker_fsm() {
        while (true) {
            (void)test_and_reconnect_camera();
            matching_array_.clear();
            sync_model_.reset();

            if (use_hardware_sync_) {
                if (!worker_fsm_resetting()) continue;
                if (!worker_fsm_matching()) continue;
                if (!worker_fsm_confirming()) continue;
                worker_fsm_locked();
            } else {
                worker_fsm_free_running();
            }
        }
    }

    [[nodiscard]] auto worker_fsm_resetting() -> bool {
        RCLCPP_INFO(logger_, "[RESETTING] Waiting for bus idle...");
        if (!worker_set_check_camera_trigger_mode(true)) return false;

        using namespace std::chrono_literals;
        while (worker_wait_until(
            [this] noexcept {
                return unmatched_signal_buffer_.readable() || unmatched_image_buffer_.readable();
            },
            std::chrono::steady_clock::now() + 100ms)) {
            unmatched_signal_buffer_.clear();
            unmatched_image_buffer_.clear();
            if (test_and_reconnect_camera()) return false;
        }

        return true;
    }

    [[nodiscard]] auto worker_fsm_matching() -> bool {
        RCLCPP_INFO(logger_, "[MATCHING] Matching signals and images...");
        if (!worker_set_check_camera_trigger_mode(false)) return false;

        const auto matching_start = std::chrono::steady_clock::now();
        using namespace std::chrono_literals;
        while (worker_wait_until(
            [this] noexcept {
                return unmatched_signal_buffer_.readable() && unmatched_image_buffer_.readable();
            },
            matching_start + 2s)) {
            matching_array_.emplace_back(consume_frame());
            if (test_and_reconnect_camera()) return false;
        }

        return true;
    }

    [[nodiscard]] auto worker_fsm_confirming() -> bool {
        RCLCPP_INFO(logger_, "[CONFIRMING] Draining buffers before fitting...");
        if (!worker_set_check_camera_trigger_mode(true)) return false;

        using namespace std::chrono_literals;
        while (worker_wait_until(
            [this] noexcept {
                return unmatched_signal_buffer_.readable() && unmatched_image_buffer_.readable();
            },
            std::chrono::steady_clock::now() + 100ms)) {
            matching_array_.emplace_back(consume_frame());
            if (test_and_reconnect_camera()) return false;
        }

        if (const auto signal_readable = unmatched_signal_buffer_.readable(),
            image_readable             = unmatched_image_buffer_.readable();
            signal_readable || image_readable) {
            RCLCPP_ERROR(logger_, "[CONFIRMING] Buffer count mismatch after drain: %zu != %zu",
                signal_readable + matching_array_.size(), image_readable + matching_array_.size());
            return false;
        }

        if (matching_array_.size() < 50) {
            RCLCPP_ERROR(logger_,
                "[CONFIRMING] Too few frames matched: collected (%zu) < minimum (50)",
                matching_array_.size());
            return false;
        }

        auto ls_result = sync_model_.initialize_from_least_squares(matching_array_);
        if (!ls_result) {
            RCLCPP_ERROR(logger_, "[CONFIRMING] Least-squares fit failed to initialize lock model");
            return false;
        }

        if (ls_result->a <= 0.8 || ls_result->a >= 1.2) {
            RCLCPP_ERROR(logger_, "[CONFIRMING] LS slope out of range: a=%.3f (expected 0.8..1.2)",
                ls_result->a);
            return false;
        }

        if (ls_result->max_abs_residual_sec >= sync_model_.residual_threshold_sec()) {
            RCLCPP_ERROR(logger_, "[CONFIRMING] LS max residual=%.3f ms >= threshold=%.3f ms",
                ls_result->max_abs_residual_sec * 1000.0,
                sync_model_.residual_threshold_sec() * 1000.0);
            return false;
        }

        RCLCPP_INFO(logger_,
            "[CONFIRMING] Locked from %zu frames: a=%.3f, b=%.3f, max_residual=%.3f ms",
            matching_array_.size(), ls_result->a, ls_result->b,
            ls_result->max_abs_residual_sec * 1000.0);
        return true;
    }

    void worker_fsm_locked() {
        if (!worker_set_check_camera_trigger_mode(false)) return;

        auto locked_frame_count        = std::size_t { 0 };
        auto last_locked_frame_id      = std::uint32_t { 0 };
        auto max_residual_sec          = 0.0;
        auto last_locked_summary_time  = std::chrono::steady_clock::now();
        auto last_locked_summary_count = std::size_t { 0 };

        while (true) {
            if (test_and_reconnect_camera()) return;

            using namespace std::chrono_literals;
            if (!worker_wait_until(
                    [this] noexcept {
                        return unmatched_signal_buffer_.readable()
                            && unmatched_image_buffer_.readable();
                    },
                    std::chrono::steady_clock::now() + 50ms)) {
                continue;
            }

            const auto next_frame_id = unmatched_image_buffer_.peek_front()->frame_id;
            if (locked_frame_count) {
                const auto expected_frame_id = last_locked_frame_id + 1;

                if (next_frame_id < expected_frame_id) {
                    RCLCPP_ERROR(logger_,
                        "[LOCKED] Frame ID moved backward: expected >= %u, got %u",
                        expected_frame_id, next_frame_id);
                    return;
                }

                const auto frame_gap = next_frame_id - expected_frame_id;
                if (frame_gap > 0) {
                    static constexpr std::uint32_t allowed_frame_id_gap = 2;
                    if (frame_gap > allowed_frame_id_gap) {
                        RCLCPP_ERROR(logger_,
                            "[LOCKED] Frame ID gap too large: expected %u, got %u, gap %u > %u",
                            expected_frame_id, next_frame_id, frame_gap, allowed_frame_id_gap);
                        return;
                    }

                    if (const auto readable = unmatched_signal_buffer_.readable();
                        readable < static_cast<std::size_t>(frame_gap) + 1) {
                        RCLCPP_ERROR(logger_,
                            "[LOCKED] Unable to tolerate camera frame gap: expected %u signals, "
                            "got %zu",
                            frame_gap + 1, readable);
                        return;
                    }
                    unmatched_signal_buffer_.pop_front_n(
                        [](UnmatchedSignal&&) noexcept { }, frame_gap);

                    RCLCPP_WARN(logger_,
                        "[LOCKED] Tolerating camera frame gap: expected %u, got %u, dropped %u "
                        "queued signals",
                        expected_frame_id, next_frame_id, frame_gap);
                }
            }

            const auto timestamp_pair = consume_frame();
            locked_frame_count++;
            last_locked_frame_id = next_frame_id;
            if (const auto now = std::chrono::steady_clock::now();
                now - last_locked_summary_time >= std::chrono::milliseconds { 5000 }) {
                const auto window_frames = locked_frame_count - last_locked_summary_count;
                const auto fps           = static_cast<double>(window_frames)
                    / std::chrono::duration<double>(now - last_locked_summary_time).count();
                RCLCPP_INFO(logger_,
                    "[LOCKED] %zu frames total, avg %.1f FPS, a=%.3f, b=%.3f, max_residual=%.3f ms",
                    locked_frame_count, fps, sync_model_.a(), sync_model_.b(),
                    max_residual_sec * 1000.0);
                last_locked_summary_count = locked_frame_count;
                max_residual_sec          = 0.0;
                last_locked_summary_time  = now;
            }
            const auto residual = sync_model_.residual_for(
                timestamp_pair.camera_timestamp_sec, timestamp_pair.board_timestamp_sec);
            max_residual_sec = std::max(max_residual_sec, std::abs(residual));
            if (!(std::abs(residual) < sync_model_.residual_threshold_sec())) {
                RCLCPP_ERROR(logger_,
                    "[LOCKED] RLS residual too large: residual=%.3f ms, threshold=%.3f ms, "
                    "camera_ts=%.3f ms, signal_ts=%.3f ms",
                    residual * 1000.0, sync_model_.residual_threshold_sec() * 1000.0,
                    timestamp_pair.camera_timestamp_sec * 1000.0,
                    timestamp_pair.board_timestamp_sec * 1000.0);
                return;
            }

            const auto updated_residual = sync_model_.update(
                timestamp_pair.camera_timestamp_sec, timestamp_pair.board_timestamp_sec);
            if (!std::isfinite(updated_residual)) {
                RCLCPP_ERROR(logger_, "[LOCKED] RLS update failed: non-finite numerical values");
                return;
            }
        }
    }

    void worker_fsm_free_running() {
        RCLCPP_INFO(logger_, "[FALLBACK] Entering free-running camera mode");

        if (!worker_set_check_camera_trigger_mode(false)) return;

        auto frame_count        = std::size_t { 0 };
        auto last_summary_time  = std::chrono::steady_clock::now();
        auto last_summary_count = std::size_t { 0 };

        while (true) {
            if (test_and_reconnect_camera()) {
                if (!worker_set_check_camera_trigger_mode(false)) return;
                continue;
            }

            using namespace std::chrono_literals;
            if (!worker_wait_until([this] noexcept { return unmatched_image_buffer_.readable(); },
                    std::chrono::steady_clock::now() + 50ms)) {
                continue;
            }

            consume_frame_fallback();

            ++frame_count;
            if (const auto now = std::chrono::steady_clock::now();
                now - last_summary_time >= std::chrono::milliseconds { 5000 }) {
                const auto window_frames = frame_count - last_summary_count;
                const auto fps           = static_cast<double>(window_frames)
                    / std::chrono::duration<double>(now - last_summary_time).count();
                RCLCPP_INFO(logger_, "[FALLBACK] %zu frames total, avg %.1f FPS", frame_count, fps);
                last_summary_count = frame_count;
                last_summary_time  = now;
            }
        }
    }

    [[nodiscard]] auto consume_frame() -> LinearSyncModel::TimestampPair {
        auto frame_id       = std::uint32_t { 0 };
        auto timestamp_pair = LinearSyncModel::TimestampPair { };
        auto output_frame   = std::shared_ptr<OutputFrame> { };

        if (!unmatched_image_buffer_.pop_front([&](UnmatchedImage&& image) noexcept {
                frame_id = image.frame_id;
                timestamp_pair.camera_timestamp_sec =
                    std::chrono::duration<double> { image.timestamp.time_since_epoch() }.count();
                output_frame = std::move(image.frame);
            })) {
            RCLCPP_FATAL(logger_, "Image buffer unexpectedly empty");
            std::terminate();
        }

        auto exposure_midpoint = rmcs_msgs::BoardClock::time_point { };
        if (!unmatched_signal_buffer_.pop_front([&](UnmatchedSignal&& signal) noexcept {
                timestamp_pair.board_timestamp_sec =
                    std::chrono::duration<double> { signal.board_timestamp.time_since_epoch() }
                        .count();
                exposure_midpoint                = signal.board_timestamp + half_exposure_time_;
                output_frame->exposure_timestamp = signal.host_timestamp;
            })) {
            RCLCPP_FATAL(logger_, "Signal buffer unexpectedly empty");
            std::terminate();
        }

        if (auto imu_snapshot = imu_buffer_.pop(exposure_midpoint)) {
            output_frame->imu_snapshot           = imu_snapshot->orientation;
            output_frame->gyro_body              = imu_snapshot->gyro_body;
            output_frame->sync_publish_timestamp = std::chrono::steady_clock::now();

            const auto src = cv::Mat { OutputFrame::kHeight, OutputFrame::kWidth, CV_8UC1,
                reinterpret_cast<char*>(output_frame->data_raw.data()) };
            auto mat       = cv::Mat { OutputFrame::kHeight, OutputFrame::kWidth, CV_8UC3,
                reinterpret_cast<char*>(output_frame->data.data()) };
            cv::demosaicing(src, mat, output_frame->opencv_cvt_color_code);

            frame_output_.emit(output_frame);
        } else {
            RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 1000,
                "Dropping frame #%u: no IMU snapshot available for exposure board timestamp %lld "
                "ticks",
                frame_id, static_cast<long long>(exposure_midpoint.time_since_epoch().count()));
        }

        return timestamp_pair;
    }

    void consume_frame_fallback() {
        auto frame_id     = std::uint32_t { 0 };
        auto output_frame = std::shared_ptr<OutputFrame> { };

        if (!unmatched_image_buffer_.pop_front([&](UnmatchedImage&& image) noexcept {
                frame_id     = image.frame_id;
                output_frame = std::move(image.frame);
            })) {
            RCLCPP_FATAL(logger_, "Image buffer unexpectedly empty");
            std::terminate();
        }

        const auto compensated_timestamp = output_frame->image_reception_timestamp - imu_delay_;
        output_frame->exposure_timestamp = compensated_timestamp;

        if (auto imu_snapshot = imu_buffer_.pop_host(compensated_timestamp)) {
            output_frame->imu_snapshot           = imu_snapshot->orientation;
            output_frame->gyro_body              = imu_snapshot->gyro_body;
            output_frame->sync_publish_timestamp = std::chrono::steady_clock::now();

            const auto src = cv::Mat { OutputFrame::kHeight, OutputFrame::kWidth, CV_8UC1,
                reinterpret_cast<char*>(output_frame->data_raw.data()) };
            auto mat       = cv::Mat { OutputFrame::kHeight, OutputFrame::kWidth, CV_8UC3,
                reinterpret_cast<char*>(output_frame->data.data()) };
            cv::demosaicing(src, mat, output_frame->opencv_cvt_color_code);

            frame_output_.emit(output_frame);
        } else {
            RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 1000,
                "Dropping frame #%u: no IMU snapshot available in fallback mode", frame_id);
        }
    }

    [[nodiscard]] auto test_and_reconnect_camera() -> bool {
        if (camera_) {
            if (const auto fault_message = camera_->take_transport_fault_message()) {
                RCLCPP_ERROR(logger_, "[CAMERA] Runtime fault 0x%08x (%s), reconnecting...",
                    *fault_message, Hikcamera::sdk_transport_exception_to_string(*fault_message));
            } else if (auto frame_age = std::chrono::steady_clock::now()
                    - last_frame_time_.load(std::memory_order::relaxed);
                frame_age > std::chrono::milliseconds { 5000 }) {
                RCLCPP_ERROR(logger_,
                    "[CAMERA] Frame watchdog timeout: no frame for %ld ms, reconnecting...",
                    std::chrono::duration_cast<std::chrono::milliseconds>(frame_age).count());
            } else {
                return false;
            }
        }
        camera_.reset();

        for (auto attempt_count = 1;; attempt_count++) {
            try {
                camera_          = std::make_unique<Hikcamera>(camera_config_,
                    [this](const Hikcamera::Frame& frame) noexcept { frame_callback(frame); });
                last_frame_time_ = std::chrono::steady_clock::now();
                RCLCPP_INFO(logger_, "[CAMERA] Connected successfully");

                break;
            } catch (const std::runtime_error& error) {
                RCLCPP_ERROR(logger_, "[CAMERA] Connect attempt #%d: %s, retry in 2500 ms",
                    attempt_count, error.what());
                using namespace std::chrono_literals;
                worker_sleep_for(2500ms);
            }
        }

        return true;
    }

    [[nodiscard]] auto worker_set_check_camera_trigger_mode(bool soft) -> bool {
        const auto code = static_cast<std::uint32_t>(camera_->set_soft_trigger(soft));

        if (code != MV_OK) {
            RCLCPP_ERROR(logger_, "[CAMERA] Set trigger mode failed: 0x%08x (%s)", code,
                Hikcamera::sdk_error_to_string(code));

            camera_.reset();
            using namespace std::chrono_literals;
            worker_sleep_for(2500ms);

            return false;
        }

        return true;
    }

    void worker_sleep_for(std::chrono::steady_clock::duration time) {
        worker_wait_until([] noexcept { return false; }, std::chrono::steady_clock::now() + time);
    }

    template <typename FunctorT>
        requires(std::is_nothrow_invocable_v<FunctorT>)
    auto worker_wait_until(FunctorT&& check, std::chrono::steady_clock::time_point deadline)
        -> bool {
        while (true) {
            const auto old = event_count_.load(std::memory_order::acquire);
            if (stop_requested_.test(std::memory_order::relaxed)) throw StopRequestException { };
            if (check()) return true;
            if (!rmcs_utility::atomic_futex_wait_until(
                    event_count_, old, deadline, std::memory_order::acquire)) {
                return check();
            }
        }
    }

private:
    static constexpr std::size_t kExpectedFrameWidth =
        static_cast<std::size_t>(Hikcamera::Cs016FrameWidth);
    static constexpr std::size_t kExpectedFrameHeight =
        static_cast<std::size_t>(Hikcamera::Cs016FrameHeight);
    static constexpr std::size_t kExpectedFrameSize = kExpectedFrameWidth * kExpectedFrameHeight;

    const rclcpp::Logger logger_ { get_logger() };
    const Hikcamera::CameraConfig camera_config_;

    using OutputFrame = rmcs_msgs::CameraFrame;

    EventInputInterface<rmcs_msgs::BoardClock::time_point> signal_input_ {
        [this](const rmcs_msgs::BoardClock::time_point& timestamp) { signal_callback(timestamp); },
    };

    struct UnmatchedSignal {
        rmcs_msgs::BoardClock::time_point board_timestamp;
        std::chrono::steady_clock::time_point host_timestamp;
    };
    rmcs_utility::RingBuffer<UnmatchedSignal> unmatched_signal_buffer_ { 32 };

    std::unique_ptr<Hikcamera> camera_;
    std::atomic<std::chrono::steady_clock::time_point> last_frame_time_ =
        std::chrono::steady_clock::time_point::min();

    rmcs_utility::PooledSharedFactory<OutputFrame> output_frame_factory_ { 128 };
    struct UnmatchedImage {
        std::shared_ptr<OutputFrame> frame;
        std::uint32_t frame_id;
        Hikcamera::HikDeviceClock::time_point timestamp;
    };
    rmcs_utility::RingBuffer<UnmatchedImage> unmatched_image_buffer_ { 16 };

    const rmcs_msgs::BoardClock::duration half_exposure_time_;
    const std::chrono::steady_clock::duration imu_delay_;
    ImuSnapshotBuffer imu_buffer_ { *this, 1024 };
    EventOutputInterface<std::shared_ptr<const OutputFrame>> frame_output_;

    std::atomic<std::uint32_t> event_count_ = 0;

    std::vector<LinearSyncModel::TimestampPair> matching_array_;
    LinearSyncModel sync_model_;

    bool use_hardware_sync_;

    class StopRequestException : public std::exception {
    public:
        auto what() const noexcept -> const char* override { return "Stop requested"; }
    };
    std::atomic_flag stop_requested_ = ATOMIC_FLAG_INIT;

    std::thread worker_thread_;
};

} // namespace rmcs

PLUGINLIB_EXPORT_CLASS(rmcs::AutoAimCapturerComponent, rmcs_executor::Component)
