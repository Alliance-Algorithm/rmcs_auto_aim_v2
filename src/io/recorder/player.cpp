#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <eigen3/Eigen/Geometry>
#include <opencv2/imgproc.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/camera_frame.hpp>

#include "utility/csv/csv_reader.hpp"
#include "utility/serializable.hpp"
#include "utility/service.hpp"

namespace rmcs {

class AutoAimPlayerComponent final : public rmcs_executor::Component, public rclcpp::Node {
public:
    AutoAimPlayerComponent()
        : Node(get_component_name(),
              rclcpp::NodeOptions { }.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger())
        , input_path_(get_parameter_or<std::string>("input_path", ""))
        , loop_play_(get_parameter_or<bool>("loop_play", false))
        , pause_period_([this]() {
            const auto pause_fps = get_parameter_or<double>("pause_fps", 30.0);
            if (!std::isfinite(pause_fps) || pause_fps < 1.0 || pause_fps > 1000.0)
                throw std::runtime_error("Parameter \"pause_fps\" contains illegal value");
            return std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double> { 1.0 / pause_fps });
        }()) {

        if (input_path_.empty()) {
            throw std::runtime_error("Parameter \"input_path\" must not be empty");
        }

        {
            const auto raw_path = input_path_ / "record.raw";
            raw_stream_.open(raw_path, std::ios::binary | std::ios::in);
            if (!raw_stream_.is_open())
                throw std::runtime_error { "Failed to open raw input file: " + raw_path.string() };
        }
        parse_csv();

        register_output("/gimbal/auto_aim/camera_frame", frame_output_);
    }

    ~AutoAimPlayerComponent() override {
        stop_requested_.store(true, std::memory_order::relaxed);
        if (worker_thread_.joinable()) worker_thread_.join();
        if (service_thread_.joinable()) service_thread_.join();
    }

    void before_updating() override {
        RCLCPP_INFO(logger_, "Playing CameraFrame stream from %s", input_path_.c_str());
        worker_thread_  = std::thread { [this] { worker_main(); } };
        service_thread_ = std::thread { [this] { service_main(); } };
    }

    void update() override { }

private:
    struct FrameMetadata : util::Serializable {
        std::int64_t raw_offset_bytes;
        std::int64_t raw_size_bytes;
        std::uint32_t width;
        std::uint32_t height;
        int opencv_cvt_color_code;
        std::int64_t exposure_timestamp_ns;
        std::int64_t image_reception_timestamp_ns;
        std::int64_t sync_publish_timestamp_ns;
        double imu_quat_w;
        double imu_quat_x;
        double imu_quat_y;
        double imu_quat_z;

        static constexpr std::tuple metas {
            &FrameMetadata::raw_offset_bytes,
            "raw_offset_bytes",
            &FrameMetadata::raw_size_bytes,
            "raw_size_bytes",
            &FrameMetadata::width,
            "width",
            &FrameMetadata::height,
            "height",
            &FrameMetadata::opencv_cvt_color_code,
            "opencv_cvt_color_code",
            &FrameMetadata::exposure_timestamp_ns,
            "exposure_timestamp_ns",
            &FrameMetadata::image_reception_timestamp_ns,
            "image_reception_timestamp_ns",
            &FrameMetadata::sync_publish_timestamp_ns,
            "sync_publish_timestamp_ns",
            &FrameMetadata::imu_quat_w,
            "imu_quat_w",
            &FrameMetadata::imu_quat_x,
            "imu_quat_x",
            &FrameMetadata::imu_quat_y,
            "imu_quat_y",
            &FrameMetadata::imu_quat_z,
            "imu_quat_z",
        };
    };

    void parse_csv() {
        auto csv_reader      = util::CsvReader { input_path_ / "record.csv" };
        auto csv_line_number = std::size_t { 2 };
        frames_.clear();

        while (true) {
            if (csv_reader.corrupted_line()) {
                throw std::runtime_error(
                    "CSV line " + std::to_string(csv_line_number) + " has unexpected column count");
            }
            if (csv_reader.eof()) break;

            frames_.emplace_back();
            auto& metadata = frames_.back();
            if (auto result = metadata.serialize(csv_reader); !result) {
                throw std::runtime_error("Failed to parse CSV line "
                    + std::to_string(csv_line_number) + ": " + result.error());
            }

            ++csv_line_number;
            if (!csv_reader.next()) {
                if (csv_reader.corrupted_line()) {
                    throw std::runtime_error("CSV line " + std::to_string(csv_line_number)
                        + " has unexpected column count");
                }
                if (csv_reader.eof()) break;
            }
        }
    }

    void worker_main() {
        try {
            play_stream();
        } catch (const std::exception& exception) {
            RCLCPP_FATAL(
                logger_, "Player worker thread terminated by exception: %s", exception.what());
            rclcpp::shutdown();
        } catch (...) {
            RCLCPP_FATAL(logger_, "Player worker thread terminated by unknown exception");
            rclcpp::shutdown();
        }
    }

    void play_stream() {
        if (frames_.empty()) {
            RCLCPP_WARN(logger_, "Player input CSV contains no frame rows");
            return;
        }

        raw_stream_.clear();
        raw_stream_.seekg(0);

        auto now            = std::chrono::steady_clock::now();
        played_count_       = 0;
        skipped_count_      = 0;
        last_report_time_   = now;
        last_report_played_ = 0;

        std::size_t current_index = 0;
        auto frame_emit_time      = now + pause_period_;

        for (; !stop_requested_.load(std::memory_order::relaxed);
            frame_emit_time += advance_index(current_index)) {

            now = std::chrono::steady_clock::now();
            if (now > frame_emit_time) {
                ++skipped_count_;
                continue;
            }
            const auto& current_frame = frames_[current_index];

            auto output_frame = read_frame(current_frame, frame_emit_time);
            std::this_thread::sleep_until(frame_emit_time);

            if (output_frame) {
                ++played_count_;
                frame_output_.emit(output_frame);
            } else {
                ++skipped_count_;
            }

            log_due_report(now);
        }

        RCLCPP_INFO(
            logger_, "Player finished: played=%zu skipped=%zu", played_count_, skipped_count_);
    }

    [[nodiscard]] auto advance_index(std::size_t& index) -> std::chrono::steady_clock::duration {
        const auto frame_count = static_cast<std::int64_t>(frames_.size());

        const auto control_pause = control_pause_.load(std::memory_order::relaxed);
        const auto control_step  = control_step_.exchange(0, std::memory_order::relaxed);

        const auto delta = static_cast<std::int64_t>(control_step) + !control_pause;
        auto next        = static_cast<std::int64_t>(index) + delta;

        bool repeating_tail = false;
        if (loop_play_) {
            next %= frame_count;
            if (next < 0) next += frame_count;
        } else {
            if (next < 0) {
                next = 0;
            } else if (next >= frame_count) {
                next           = frame_count - 1;
                repeating_tail = true;
            }
        }

        index = static_cast<std::size_t>(next);
        return (control_pause || index == 0 || repeating_tail)
            ? pause_period_
            : std::chrono::nanoseconds(frames_[index].sync_publish_timestamp_ns
                  - frames_[index - 1].sync_publish_timestamp_ns);
    }

    [[nodiscard]] auto read_frame(
        const FrameMetadata& metadata, std::chrono::steady_clock::time_point emit_time)
        -> std::shared_ptr<rmcs_msgs::CameraFrame> {
        if (metadata.raw_size_bytes != static_cast<std::int64_t>(rmcs_msgs::CameraFrame::kFrameSize)
            || metadata.width != rmcs_msgs::CameraFrame::kWidth
            || metadata.height != rmcs_msgs::CameraFrame::kHeight) {
            RCLCPP_WARN(logger_, "Skipping frame with unsupported dimensions: %u×%u, raw_size=%lld",
                metadata.width, metadata.height, static_cast<long long>(metadata.raw_size_bytes));
            return nullptr;
        }

        raw_stream_.clear();
        raw_stream_.seekg(static_cast<std::streamoff>(metadata.raw_offset_bytes), std::ios::beg);
        if (!raw_stream_) {
            RCLCPP_WARN(logger_, "Player failed to seek to raw frame offset; skipping frame");
            return nullptr;
        }

        auto frame = std::make_shared<rmcs_msgs::CameraFrame>();
        raw_stream_.read(reinterpret_cast<char*>(frame->data_raw.data()),
            static_cast<std::streamsize>(frame->data_raw.size()));
        if (!raw_stream_) {
            RCLCPP_WARN(logger_, "Player reached incomplete raw frame data; skipping frame");
            return nullptr;
        }

        frame->opencv_cvt_color_code = metadata.opencv_cvt_color_code;
        frame->imu_snapshot          = Eigen::Quaterniond {
            metadata.imu_quat_w,
            metadata.imu_quat_x,
            metadata.imu_quat_y,
            metadata.imu_quat_z,
        };

        const auto restore_timestamp = [emit_time](std::int64_t timestamp_ns,
                                           std::int64_t base_sync_timestamp_ns) {
            return emit_time + std::chrono::nanoseconds { timestamp_ns - base_sync_timestamp_ns };
        };
        frame->exposure_timestamp =
            restore_timestamp(metadata.exposure_timestamp_ns, metadata.sync_publish_timestamp_ns);
        frame->image_reception_timestamp = restore_timestamp(
            metadata.image_reception_timestamp_ns, metadata.sync_publish_timestamp_ns);
        frame->sync_publish_timestamp = emit_time;

        const auto src = cv::Mat { rmcs_msgs::CameraFrame::kHeight, rmcs_msgs::CameraFrame::kWidth,
            CV_8UC1, reinterpret_cast<char*>(frame->data_raw.data()) };
        auto mat       = cv::Mat { rmcs_msgs::CameraFrame::kHeight, rmcs_msgs::CameraFrame::kWidth,
            CV_8UC3, reinterpret_cast<char*>(frame->data.data()) };
        cv::demosaicing(src, mat, frame->opencv_cvt_color_code);

        return frame;
    }

    void log_due_report(std::chrono::steady_clock::time_point now) {
        if (now - last_report_time_ < std::chrono::seconds { 5 }) return;

        const auto window_played = played_count_ - last_report_played_;
        const auto total_frames  = played_count_ + skipped_count_;
        const auto skipped_ratio = total_frames == 0
            ? 0.0
            : static_cast<double>(skipped_count_) / static_cast<double>(total_frames) * 100.0;
        const auto elapsed_sec   = std::chrono::duration<double>(now - last_report_time_).count();
        const auto window_fps =
            elapsed_sec <= 0.0 ? 0.0 : static_cast<double>(window_played) / elapsed_sec;
        RCLCPP_INFO(logger_, "Played/Skipped: %zu/%zu (%.1f%%), Window FPS: %.1f", played_count_,
            skipped_count_, skipped_ratio, window_fps);

        last_report_time_   = now;
        last_report_played_ = played_count_;
    }

    void service_main() {
        auto service = util::make_service<"player">(
            util::make_action<"pause", bool>(
                [this](bool value) { control_pause_.store(value, std::memory_order::relaxed); }),
            util::make_action<"step", int>([this](int delta) {
                if (delta == 0) return;
                control_step_.fetch_add(delta, std::memory_order::relaxed);
            }));

        while (!stop_requested_.load(std::memory_order::relaxed)) {
            service.spin_once();
            service.update_context(
                "paused", control_pause_.load(std::memory_order::relaxed) ? "1" : "0");
            std::this_thread::sleep_for(std::chrono::milliseconds { 100 });
        }
    }

private: // constants
    const rclcpp::Logger logger_;
    const std::filesystem::path input_path_;
    const bool loop_play_;
    const std::chrono::steady_clock::duration pause_period_;

private: // frame
    std::ifstream raw_stream_;
    std::vector<FrameMetadata> frames_;

private: // report
    std::size_t played_count_  = 0;
    std::size_t skipped_count_ = 0;
    std::chrono::steady_clock::time_point last_report_time_;
    std::size_t last_report_played_ = 0;

private: // control
    std::atomic<bool> control_pause_ { false };
    std::atomic<int> control_step_ { 0 };

private: // output
    EventOutputInterface<std::shared_ptr<const rmcs_msgs::CameraFrame>> frame_output_;

private: // worker
    std::atomic<bool> stop_requested_ { false };
    std::thread worker_thread_;
    std::thread service_thread_;
};

} // namespace rmcs

PLUGINLIB_EXPORT_CLASS(rmcs::AutoAimPlayerComponent, rmcs_executor::Component)
