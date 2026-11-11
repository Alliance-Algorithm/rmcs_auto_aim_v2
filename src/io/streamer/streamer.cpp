#include <chrono>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/camera_frame.hpp>

#include "module/debug/visualization/stream_context.hpp"

namespace rmcs {

// Streams CameraFrame events without running the auto-aim pipeline.
class AutoAimStreamerComponent final : public rmcs_executor::Component, public rclcpp::Node {
public:
    AutoAimStreamerComponent()
        : Node(get_component_name(),
              rclcpp::NodeOptions { }.automatically_declare_parameters_from_overrides(true)) {
        const auto frame_topic =
            get_parameter_or<std::string>("frame_topic", "/gimbal/auto_aim/camera_frame");
        const auto host = get_parameter_or<std::string>("monitor_host", "127.0.0.1");
        const auto port = get_parameter_or<std::string>("monitor_port", "5000");
        const auto hz   = get_parameter_or<int>("framerate", 60);
        const auto type = get_parameter_or<std::string>("stream_type", "RTP_H264");
        if (frame_topic.empty() || host.empty() || port.empty() || hz <= 0) {
            throw std::runtime_error {
                "Stream topic, host and port must be nonempty; framerate must be positive",
            };
        }

        auto stream_type = debug::StreamContext::StreamType::RTP_H264;
        if (type == "RTP_JEPG") {
            stream_type = debug::StreamContext::StreamType::RTP_JEPG;
        } else if (type != "RTP_H264") {
            throw std::runtime_error { "stream_type must be RTP_H264 or RTP_JEPG" };
        }

        interval_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double> { 1.0 / hz });
        stream_ = std::make_unique<debug::StreamContext>(stream_type,
            debug::StreamContext::VideoFormat { static_cast<int>(rmcs_msgs::CameraFrame::kWidth),
                static_cast<int>(rmcs_msgs::CameraFrame::kHeight), hz },
            debug::StreamContext::StreamTarget { host, port });

        register_input(frame_topic, frame_input_);
    }

    void update() override { }

private:
    void process_frame(const rmcs_msgs::CameraFrame& frame) {
        const auto now = std::chrono::steady_clock::now();
        if (now < next_frame_time_) return;
        next_frame_time_ = now + interval_;

        if (!stream_->opened()) {
            if (now < next_open_time_) return;
            next_open_time_ = now + std::chrono::seconds { 2 };
            if (auto result = stream_->open(); !result) {
                RCLCPP_ERROR(logger_, "Failed to open camera stream: %s", result.error().c_str());
                return;
            }
            RCLCPP_INFO(logger_, "Camera stream opened");
        }

        // Own the pixels passed to OpenCV; never modify the shared camera frame.
        auto image = cv::Mat(static_cast<int>(rmcs_msgs::CameraFrame::kHeight),
            static_cast<int>(rmcs_msgs::CameraFrame::kWidth), CV_8UC3);
        std::memcpy(image.data, frame.data.data(), frame.data.size());
        stream_->write(image);
    }

private: // constants
    const rclcpp::Logger logger_ = get_logger();

private: // stream
    std::unique_ptr<debug::StreamContext> stream_;
    std::chrono::steady_clock::duration interval_ { };
    std::chrono::steady_clock::time_point next_frame_time_ { };
    std::chrono::steady_clock::time_point next_open_time_ { };

private: // input
    // Destroy and join the input worker before destroying its streaming context.
    QueuedEventInputInterface<std::shared_ptr<const rmcs_msgs::CameraFrame>> frame_input_ {
        2,
        [this](std::shared_ptr<const rmcs_msgs::CameraFrame>&& frame) {
            if (frame) process_frame(*frame);
        },
    };
};

} // namespace rmcs

PLUGINLIB_EXPORT_CLASS(rmcs::AutoAimStreamerComponent, rmcs_executor::Component)
