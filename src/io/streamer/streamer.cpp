#include "streamer.hpp"

#include <chrono>
#include <cstring>
#include <stdexcept>

#include <pluginlib/class_list_macros.hpp>
#include <rmcs_msgs/camera_frame.hpp>

#include "module/debug/visualization/stream_context.hpp"

namespace rmcs {

struct AutoAimStreamerComponent::Impl {
    explicit Impl(AutoAimStreamerComponent& component)
        : logger(component.get_logger()) {
        const auto frame_topic =
            component.get_parameter_or<std::string>("frame_topic", "/gimbal/auto_aim/camera_frame");
        const auto host = component.get_parameter_or<std::string>("monitor_host", "127.0.0.1");
        const auto port = component.get_parameter_or<std::string>("monitor_port", "5000");
        const auto hz   = component.get_parameter_or<int>("framerate", 60);
        const auto type = component.get_parameter_or<std::string>("stream_type", "RTP_H264");
        if (frame_topic.empty() || host.empty() || port.empty() || hz <= 0) {
            throw std::runtime_error("Stream topic, host and port must be nonempty; framerate must "
                                     "be positive");
        }
        auto stream_type = debug::StreamContext::StreamType::RTP_H264;
        if (type == "RTP_JEPG") {
            stream_type = debug::StreamContext::StreamType::RTP_JEPG;
        } else if (type != "RTP_H264") {
            throw std::runtime_error("stream_type must be RTP_H264 or RTP_JEPG");
        }
        interval =
            std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / hz));
        stream = std::make_unique<debug::StreamContext>(stream_type,
            debug::StreamContext::VideoFormat { static_cast<int>(rmcs_msgs::CameraFrame::kWidth),
                static_cast<int>(rmcs_msgs::CameraFrame::kHeight), hz },
            debug::StreamContext::StreamTarget { host, port });
        component.register_input(frame_topic, frame_input);
    }

    void process_frame(const std::shared_ptr<const rmcs_msgs::CameraFrame>& frame) {
        if (!frame) return;
        const auto now = Clock::now();
        if (now < next_frame) return;
        next_frame = now + interval;

        if (!stream->opened()) {
            if (now < next_open) return;
            next_open = now + std::chrono::seconds { 2 };
            if (auto result = stream->open(); !result) {
                RCLCPP_ERROR(logger, "Failed to open camera stream: %s", result.error().c_str());
                return;
            }
            RCLCPP_INFO(logger, "Camera stream opened");
        }

        // Own the pixels passed to OpenCV; never modify the shared camera frame.
        cv::Mat image(static_cast<int>(frame->kHeight), static_cast<int>(frame->kWidth), CV_8UC3);
        std::memcpy(image.data, frame->data.data(), frame->data.size());
        stream->write(image);
    }

    using Clock = std::chrono::steady_clock;
    rclcpp::Logger logger;
    Clock::duration interval { };
    Clock::time_point next_frame { };
    Clock::time_point next_open { };
    std::unique_ptr<debug::StreamContext> stream;

    // Destroy and join the input worker before destroying its streaming context.
    QueuedEventInputInterface<std::shared_ptr<const rmcs_msgs::CameraFrame>> frame_input { 2,
        [this](std::shared_ptr<const rmcs_msgs::CameraFrame>&& frame) { process_frame(frame); } };
};

AutoAimStreamerComponent::AutoAimStreamerComponent()
    : Node(get_component_name(),
          rclcpp::NodeOptions { }.automatically_declare_parameters_from_overrides(true))
    , pimpl(std::make_unique<Impl>(*this)) { }

AutoAimStreamerComponent::~AutoAimStreamerComponent() = default;

void AutoAimStreamerComponent::update() { }

} // namespace rmcs

PLUGINLIB_EXPORT_CLASS(rmcs::AutoAimStreamerComponent, rmcs_executor::Component)
