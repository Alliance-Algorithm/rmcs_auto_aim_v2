#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <format>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <utility>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/camera_frame.hpp>

#include "utility/service.hpp"

namespace rmcs {

class AutoAimRecorderComponent final : public rmcs_executor::Component, public rclcpp::Node {
public:
    AutoAimRecorderComponent()
        : Node(get_component_name(),
              rclcpp::NodeOptions { }.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger())
        , output_path_([this]() -> std::filesystem::path {
            auto path = get_parameter_or<std::string>("output_path", "");
            return path.empty() ? std::filesystem::path { "/tmp/autoaim/records" }
                                : std::filesystem::path { std::move(path) };
        }())
        , flush_every_n_frames_(validated_positive_parameter("flush_every_n_frames", 64))
        , max_duration_seconds_(get_parameter_or<std::int64_t>("max_duration_seconds", 0))
        , max_videos_size_bytes_([this]() -> std::uintmax_t {
            const auto gb = get_parameter_or<double>("max_videos_size_gb", 0.0);
            if (gb < 0.0) throw std::runtime_error("Parameter \"max_videos_size_gb\" must be >= 0");
            return static_cast<std::uintmax_t>(gb * 1024.0 * 1024.0 * 1024.0);
        }())
        , frame_input_(validated_positive_parameter("queue_depth", 16),
              [this](std::shared_ptr<const rmcs_msgs::CameraFrame>&& frame) {
                  if (frame) process_frame(*frame);
              }) {
        register_input(std::string { kFrameTopic }, frame_input_);

        if (max_duration_seconds_.count() < 0) {
            throw std::runtime_error("Parameter \"max_duration_seconds\" must be >= 0");
        }
    }

    ~AutoAimRecorderComponent() override {
        desired_recording_.store(false, std::memory_order_release);
        stop_service_requested_.store(true, std::memory_order::relaxed);
        if (service_thread_.joinable()) service_thread_.join();
    }

    void before_updating() override {
        {
            auto lock = std::lock_guard { status_mutex_ };
            status_filename_.clear();
            status_enabled_ = false;
            status_dirty_   = true;
        }
        service_thread_ = std::thread { [this] { service_main(); } };

        RCLCPP_INFO(logger_, "Recorder ready. Output base: %s (use recorder.start to begin)",
            output_path_.c_str());
    }

    void update() override { }

private:
    static constexpr std::string_view kFrameTopic = "/gimbal/auto_aim/camera_frame";
    static constexpr std::string_view kCsvHeader  = "raw_offset_bytes,raw_size_bytes,width,height,"
                                                    "opencv_cvt_color_code,"
                                                    "exposure_timestamp_ns,image_reception_"
                                                    "timestamp_ns,sync_publish_timestamp_ns,"
                                                    "imu_quat_w,imu_quat_x,imu_quat_y,imu_quat_z";

    void process_frame(const rmcs_msgs::CameraFrame& frame) {
        const auto desired_recording = desired_recording_.load(std::memory_order_acquire);
        if (desired_recording && !recording_open_) {
            if (!start_session()) {
                desired_recording_.store(false, std::memory_order_release);
                return;
            }
        }

        if (!desired_recording && recording_open_) {
            stop_session();
        }

        if (!recording_open_) return;

        if (max_duration_seconds_.count() > 0
            && std::chrono::steady_clock::now() - recording_start_time_ >= max_duration_seconds_) {
            desired_recording_.store(false, std::memory_order_release);
            stop_session();
            RCLCPP_INFO(logger_, "Recording auto-stopped: max duration (%ld s) reached",
                max_duration_seconds_.count());
            return;
        }

        write_frame_bytes(frame);
    }

    void write_frame_bytes(const rmcs_msgs::CameraFrame& frame) {
        const auto raw_offset = static_cast<std::int64_t>(raw_stream_.tellp());
        raw_stream_.write(reinterpret_cast<const char*>(frame.data_raw.data()),
            static_cast<std::streamsize>(frame.data_raw.size()));
        if (!raw_stream_) throw std::runtime_error("Failed to write raw frame bytes");

        csv_stream_ << raw_offset << ',' << frame.data_raw.size() << ',' << frame.kWidth << ','
                    << frame.kHeight << ',' << frame.opencv_cvt_color_code << ','
                    << std::chrono::duration_cast<std::chrono::nanoseconds>(
                           frame.exposure_timestamp.time_since_epoch())
                           .count()
                    << ','
                    << std::chrono::duration_cast<std::chrono::nanoseconds>(
                           frame.image_reception_timestamp.time_since_epoch())
                           .count()
                    << ','
                    << std::chrono::duration_cast<std::chrono::nanoseconds>(
                           frame.sync_publish_timestamp.time_since_epoch())
                           .count()
                    << ',' << frame.imu_snapshot.w() << ',' << frame.imu_snapshot.x() << ','
                    << frame.imu_snapshot.y() << ',' << frame.imu_snapshot.z() << '\n';
        if (!csv_stream_) throw std::runtime_error("Failed to write CSV frame metadata");

        ++recorded_frame_count_;

        if (++unflushed_frames_ >= flush_every_n_frames_) flush_streams();
    }

    void flush_streams() {
        if (raw_stream_.is_open()) {
            raw_stream_.flush();
            if (!raw_stream_) throw std::runtime_error("Failed to flush raw output file");
        }
        if (csv_stream_.is_open()) {
            csv_stream_.flush();
            if (!csv_stream_) throw std::runtime_error("Failed to flush CSV output file");
        }
        unflushed_frames_ = 0;
    }

    void service_main() {
        auto service =
            util::make_service<"recorder">(util::make_action<"start">([this] { request_start(); }),
                util::make_action<"stop">([this] { request_stop(); }));

        while (!stop_service_requested_.load(std::memory_order::relaxed)) {
            service.spin_once();

            auto filename   = std::string { };
            auto enabled    = false;
            auto has_update = false;
            {
                auto lock = std::lock_guard { status_mutex_ };
                if (status_dirty_) {
                    status_dirty_ = false;
                    filename      = status_filename_;
                    enabled       = status_enabled_;
                    has_update    = true;
                }
            }
            if (has_update) {
                service.update_context("filename", filename);
                service.update_context("enabled", enabled ? "1" : "0");
            }

            std::this_thread::sleep_for(std::chrono::milliseconds { 100 });
        }
    }

    void request_start() { desired_recording_.store(true, std::memory_order_release); }

    void request_stop() { desired_recording_.store(false, std::memory_order_release); }

    [[nodiscard]] auto start_session() -> bool {
        if (max_videos_size_bytes_ > 0) {
            auto total_size = std::uintmax_t { 0 };
            try {
                for (const auto& entry :
                    std::filesystem::recursive_directory_iterator(output_path_)) {
                    if (entry.is_regular_file()) {
                        const auto& ext = entry.path().extension();
                        if (ext == ".raw" || ext == ".csv") total_size += entry.file_size();
                    }
                }
            } catch (const std::filesystem::filesystem_error&) { }
            if (total_size >= max_videos_size_bytes_) {
                RCLCPP_ERROR(logger_,
                    "Cannot start recording: total recorded file size (%.3f GB) exceeds "
                    "limit (%.3f GB)",
                    static_cast<double>(total_size) / 1024.0 / 1024.0 / 1024.0,
                    static_cast<double>(max_videos_size_bytes_) / 1024.0 / 1024.0 / 1024.0);
                return false;
            }
        }

        raw_stream_.close();
        csv_stream_.close();

        auto session_dir     = output_path_ / make_timestamp_dirname();
        const auto base_name = session_dir.filename().string();
        auto disambiguator   = std::size_t { 1 };
        while (std::filesystem::exists(session_dir)) {
            session_dir = output_path_ / std::format("{}_{}", base_name, disambiguator++);
        }
        std::filesystem::create_directories(session_dir);

        session_dir_        = std::move(session_dir);
        const auto raw_path = session_dir_ / "record.raw";
        const auto csv_path = session_dir_ / "record.csv";

        raw_stream_.imbue(std::locale::classic());
        raw_stream_.open(raw_path, std::ios::binary | std::ios::out | std::ios::trunc);
        if (!raw_stream_.is_open()) {
            RCLCPP_ERROR(logger_, "Failed to open raw output file: %s", raw_path.c_str());
            session_dir_.clear();
            return false;
        }

        csv_stream_.imbue(std::locale::classic());
        csv_stream_.open(csv_path, std::ios::out | std::ios::trunc);
        if (!csv_stream_.is_open()) {
            RCLCPP_ERROR(logger_, "Failed to open CSV output file: %s", csv_path.c_str());
            raw_stream_.close();
            session_dir_.clear();
            return false;
        }
        csv_stream_ << kCsvHeader << '\n';
        csv_stream_ << std::setprecision(std::numeric_limits<double>::max_digits10);

        recorded_frame_count_ = 0;
        unflushed_frames_     = 0;
        recording_start_time_ = std::chrono::steady_clock::now();
        recording_open_       = true;

        {
            auto lock        = std::lock_guard { status_mutex_ };
            status_filename_ = session_dir_.string();
            status_enabled_  = true;
            status_dirty_    = true;
        }

        RCLCPP_INFO(logger_, "Recording started: %s", session_dir_.c_str());
        return true;
    }

    void stop_session() {
        if (!recording_open_) return;

        flush_streams();
        raw_stream_.close();
        csv_stream_.close();
        recording_open_ = false;

        {
            auto lock       = std::lock_guard { status_mutex_ };
            status_enabled_ = false;
            status_dirty_   = true;
        }

        RCLCPP_INFO(logger_, "Recording stopped: %zu frames written to %s", recorded_frame_count_,
            session_dir_.c_str());
    }

    [[nodiscard]] static auto make_timestamp_dirname() -> std::string {
        const auto now = std::chrono::floor<std::chrono::seconds>(std::chrono::system_clock::now());
        const auto zone      = std::chrono::locate_zone("Asia/Shanghai");
        const auto zone_time = std::chrono::zoned_time { zone, now };
        return std::format("{:%Y-%m-%d_%H-%M-%S}", zone_time);
    }

    [[nodiscard]] auto validated_positive_parameter(
        const char* name, std::int64_t default_value) const -> std::size_t {
        const auto value = get_parameter_or<std::int64_t>(name, default_value);
        if (value <= 0) {
            throw std::runtime_error(std::string { "Parameter \"" } + name + "\" must be positive");
        }
        return static_cast<std::size_t>(value);
    }

private: // constants
    const rclcpp::Logger logger_;
    const std::filesystem::path output_path_;
    const std::size_t flush_every_n_frames_;
    const std::chrono::seconds max_duration_seconds_;
    const std::uintmax_t max_videos_size_bytes_;

private: // session
    std::filesystem::path session_dir_;
    std::ofstream raw_stream_;
    std::ofstream csv_stream_;
    std::size_t unflushed_frames_ = 0;
    bool recording_open_          = false;
    std::chrono::steady_clock::time_point recording_start_time_;
    std::size_t recorded_frame_count_ = 0;

private: // status
    std::mutex status_mutex_;
    std::string status_filename_;
    bool status_enabled_ = false;
    bool status_dirty_   = false;

private: // input
    QueuedEventInputInterface<std::shared_ptr<const rmcs_msgs::CameraFrame>> frame_input_;

private: // control
    std::atomic<bool> desired_recording_ { false };
    std::atomic<bool> stop_service_requested_ { false };

private: // worker
    std::thread service_thread_;
};

} // namespace rmcs

PLUGINLIB_EXPORT_CLASS(rmcs::AutoAimRecorderComponent, rmcs_executor::Component)
