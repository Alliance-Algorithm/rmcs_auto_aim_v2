#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/camera_frame.hpp>

namespace rmcs {

class AutoAimVideoPlayerComponent final : public rmcs_executor::Component, public rclcpp::Node {
public:
    AutoAimVideoPlayerComponent()
        : Node(get_component_name(),
              rclcpp::NodeOptions { }.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger())
        , input_path_(get_parameter_or<std::string>("input_path", ""))
        , frame_period_([this]() {
              const auto framerate = get_parameter_or<double>("framerate", 30.0);
              if (!std::isfinite(framerate) || framerate <= 0.0) {
                  throw std::runtime_error("Parameter \"framerate\" must be finite and > 0");
              }
              return std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                  std::chrono::duration<double> { 1.0 / framerate });
          }())
        , loop_play_(get_parameter_or<bool>("loop_play", false)) {
        if (input_path_.empty()) {
            throw std::runtime_error("Parameter \"input_path\" must not be empty");
        }
        if (!capture_.open(input_path_)) {
            throw std::runtime_error("Failed to open input video: " + input_path_);
        }

        register_output("/gimbal/auto_aim/camera_frame", frame_output_);
    }

    ~AutoAimVideoPlayerComponent() override {
        stop_requested_.store(true, std::memory_order::relaxed);
        if (worker_thread_.joinable()) worker_thread_.join();
    }

    void before_updating() override {
        RCLCPP_INFO(logger_, "Playing video stream from %s", input_path_.c_str());
        worker_thread_ = std::thread { [this] { worker_main(); } };
    }

    void update() override { }

private:
    void worker_main() {
        try {
            play_stream();
        } catch (const std::exception& exception) {
            RCLCPP_FATAL(logger_, "Video player worker thread terminated by exception: %s",
                exception.what());
            rclcpp::shutdown();
        } catch (...) {
            RCLCPP_FATAL(logger_, "Video player worker thread terminated by unknown exception");
            rclcpp::shutdown();
        }
    }

    void play_stream() {
        auto next_emit_time = std::chrono::steady_clock::now();
        while (!stop_requested_.load(std::memory_order::relaxed)) {
            auto output_frame = read_frame();
            if (!output_frame) {
                RCLCPP_INFO(logger_, "Video playback finished: %s", input_path_.c_str());
                return;
            }

            std::this_thread::sleep_until(next_emit_time);
            const auto publish_time              = std::chrono::steady_clock::now();
            output_frame->exposure_timestamp     = publish_time;
            output_frame->image_reception_timestamp = publish_time;
            output_frame->sync_publish_timestamp = publish_time;
            frame_output_.emit(output_frame);

            next_emit_time += frame_period_;
        }
    }

    [[nodiscard]] auto read_frame() -> std::shared_ptr<rmcs_msgs::CameraFrame> {
        auto frame = cv::Mat { };
        if (!capture_.read(frame) || frame.empty()) {
            if (!loop_play_) return nullptr;
            if (!capture_.set(cv::CAP_PROP_POS_FRAMES, 0) || !capture_.read(frame)
                || frame.empty()) {
                throw std::runtime_error("Failed to restart input video from beginning");
            }
        }

        auto bgr = cv::Mat { };
        switch (frame.channels()) {
        case 1:
            cv::cvtColor(frame, bgr, cv::COLOR_GRAY2BGR);
            break;
        case 3:
            bgr = frame;
            break;
        case 4:
            cv::cvtColor(frame, bgr, cv::COLOR_BGRA2BGR);
            break;
        default:
            throw std::runtime_error(
                "Unsupported input video channel count: " + std::to_string(frame.channels()));
        }

        auto normalized = cv::Mat { };
        if (bgr.type() == CV_8UC3) {
            normalized = bgr;
        } else {
            bgr.convertTo(normalized, CV_8UC3);
        }

        auto resized = cv::Mat { };
        if (normalized.cols == static_cast<int>(rmcs_msgs::CameraFrame::kWidth)
            && normalized.rows == static_cast<int>(rmcs_msgs::CameraFrame::kHeight)) {
            resized = normalized;
        } else {
            cv::resize(normalized, resized,
                cv::Size { static_cast<int>(rmcs_msgs::CameraFrame::kWidth),
                    static_cast<int>(rmcs_msgs::CameraFrame::kHeight) });
        }
        if (!resized.isContinuous()) resized = resized.clone();

        auto output_frame = std::make_shared<rmcs_msgs::CameraFrame>();
        output_frame->data_raw.fill(std::byte { 0 });
        output_frame->opencv_cvt_color_code = 0;
        output_frame->imu_snapshot          = Eigen::Quaterniond::Identity();
        std::memcpy(output_frame->data.data(), resized.data, output_frame->data.size());
        return output_frame;
    }

private: // constants
    const rclcpp::Logger logger_;
    const std::string input_path_;
    const std::chrono::steady_clock::duration frame_period_;
    const bool loop_play_;

private: // io
    cv::VideoCapture capture_;
    EventOutputInterface<std::shared_ptr<const rmcs_msgs::CameraFrame>> frame_output_;

private: // worker
    std::atomic<bool> stop_requested_ { false };
    std::thread worker_thread_;
};

} // namespace rmcs

PLUGINLIB_EXPORT_CLASS(rmcs::AutoAimVideoPlayerComponent, rmcs_executor::Component)
