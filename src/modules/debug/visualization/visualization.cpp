#include "visualization.hpp"
#include "stream_session.hpp"

#include <boost/lockfree/spsc_queue.hpp>

#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <chrono>
#include <format>
#include <thread>

using namespace rmcs::module;

struct Visualization::Impl final {
public:
    explicit Impl(utility::Node& node, bool use_streaming = false) noexcept
        : node_ { node } {

        if (use_streaming) {
            streaming_thread_ =
                std::jthread { [this](const auto& token) { streaming_thread(token); } };
        }
    }

    ~Impl() noexcept {
        streaming_thread_.request_stop();
        if (streaming_thread_.joinable()) {
            streaming_thread_.join();
        }
    }

    auto push(const cv::Mat& image) noexcept -> bool { return buffer_.push(image); }

    auto generate_test_image(uint8_t& hue) const noexcept -> cv::Mat {

        const auto hsv_scalar = cv::Scalar { static_cast<double>(hue), 255, 255 };
        const auto hsv_width  = stream_width_;
        const auto hsv_height = stream_height_;
        const auto hsv_format = CV_8UC3;
        const auto hsv_frame  = cv::Mat { hsv_height, hsv_width, hsv_format, hsv_scalar };

        auto result_frame = cv::Mat {};
        cv::cvtColor(hsv_frame, result_frame, cv::COLOR_HSV2BGR);

        hue = (hue + 1) % 180;
        return result_frame;
    }

private:
    std::jthread streaming_thread_;
    std::string stream_host_ = "127.0.0.1";
    std::string stream_port_ = "5000";
    int stream_width_        = 640;
    int stream_height_       = 480;
    int stream_fps_          = 30;

    boost::lockfree::spsc_queue<cv::Mat, boost::lockfree::capacity<30>> buffer_;
    utility::Node& node_;

private:
    auto streaming_thread(const std::stop_token& token) -> void {

        const auto pipeline = Pipeline::udpsink(
            stream_host_, stream_port_, stream_width_, stream_height_, stream_fps_);

        auto stream_sender = cv::VideoWriter(std::string { pipeline }, cv::CAP_GSTREAMER, 0,
            stream_fps_, cv::Size(stream_width_, stream_height_), true);
        if (!stream_sender.isOpened()) {
            node_.rclcpp_error("Failed to open the stream pipeline, can not show the image");
            return;
        }

        const auto interval = std::chrono::milliseconds { 1'000 / stream_fps_ };

        auto timestamp = std::chrono::steady_clock::now();

        node_.rclcpp_info("Streaming thread start: [{}]", std::string { pipeline });

        // Process loop
        while (!token.stop_requested()) {

            auto buffer_frame = cv::Mat {};
            if (buffer_.pop(buffer_frame)) stream_sender.write(buffer_frame);

            std::this_thread::sleep_until(timestamp + interval);
            timestamp = std::chrono::steady_clock::now();
        }

        node_.rclcpp_info("Streaming thread is stopping now");
    } // end thread process
};

Visualization::Visualization(utility::Node& node) noexcept
    : pimpl { std::make_unique<Impl>(node) } { }

Visualization::~Visualization() noexcept = default;

auto Visualization::streaming(const cv::Mat& image) noexcept -> bool { return pimpl->push(image); }

auto Visualization::generate_test_image(uint8_t& hue) const noexcept -> cv::Mat {
    return pimpl->generate_test_image(hue);
}

auto Visualization::block_and_test(utility::Node& node) noexcept -> void {

    auto visualization = std::make_unique<module::Visualization>(node);
    auto hue_value     = uint8_t { 0 };
    auto timestamp     = std::chrono::steady_clock::now();

    while (rclcpp::ok()) {
        const auto test_image = visualization->generate_test_image(hue_value);
        if (!visualization->streaming(test_image)) {
            node.rclcpp_warn("Pushed failed");
        }
        constexpr auto interval = std::chrono::milliseconds { 1'000 / 30 };
        std::this_thread::sleep_until(timestamp + interval);

        timestamp = std::chrono::steady_clock::now();
    }
}