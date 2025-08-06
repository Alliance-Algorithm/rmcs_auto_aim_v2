#include "streamer.hpp"
#include "stream_session.hpp"

#include <boost/lockfree/spsc_queue.hpp>

#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <chrono>
#include <format>
#include <thread>
#include <tuple>

using namespace rmcs::module;

struct Streamer::Impl final {
public:
    explicit Impl(utility::Node& node) noexcept
        : session_ { std::make_unique<StreamSession>() }
        , node_ { node } { }

    ~Impl() noexcept {
        streaming_thread_.request_stop();
        if (streaming_thread_.joinable()) {
            streaming_thread_.join();
        }
    }

    auto open(const RTP_UDP& config) -> bool {
        const auto [w, h, hz, host, port] = config;

        const auto result = session_->open_rtp(w, h, hz, host, port);
        if (result) {
            streaming_thread_ = std::jthread {
                [this](const auto& t) { streaming_thread(t); },
            };
            return true;
        }
        node_.rclcpp_error("{}", result.error());
        return false;
    }
    auto open(const RTSP_UDP& config) -> bool {
        const auto [w, h, hz, host, port] = config;

        const auto result = session_->open_rtsp(w, h, hz, host, port);
        if (result) {
            streaming_thread_ = std::jthread {
                [this](const auto& t) { streaming_thread(t); },
            };
            return true;
        }
        node_.rclcpp_error("{}", result.error());
        return false;
    }
    auto open(const RTSP_TCP& config) -> bool {
        std::ignore = config;
        std::ignore = this;
        return false;
    }

    auto send(const cv::Mat& image) noexcept -> bool { return buffer_.push(image); }

    static auto generate_test_image(uint8_t& hue) noexcept -> cv::Mat {

        const auto hsv_scalar = cv::Scalar { static_cast<double>(hue), 255, 255 };
        const auto hsv_width  = 720;
        const auto hsv_height = 540;
        const auto hsv_format = CV_8UC3;
        const auto hsv_frame  = cv::Mat { hsv_height, hsv_width, hsv_format, hsv_scalar };

        auto result_frame = cv::Mat {};
        cv::cvtColor(hsv_frame, result_frame, cv::COLOR_HSV2BGR);

        hue = (hue + 1) % 180;
        return result_frame;
    }

private:
    std::jthread streaming_thread_;
    std::unique_ptr<StreamSession> session_;

    boost::lockfree::spsc_queue<cv::Mat, boost::lockfree::capacity<30>> buffer_;
    utility::Node& node_;

private:
    auto streaming_thread(const std::stop_token& token) -> void {

        const auto interval = std::chrono::milliseconds { 1'000 / session_->hz };

        auto timestamp = std::chrono::steady_clock::now();

        node_.rclcpp_info("Streaming thread start: [{}]", std::string { session_->pipeline });

        // Process loop
        while (!token.stop_requested()) {

            auto buffer_frame = cv::Mat {};
            if (buffer_.pop(buffer_frame)) {
                session_->send(buffer_frame);

                static auto count = std::size_t { 0 };
                node_.rclcpp_info("Send {} frames", count++);
            }

            std::this_thread::sleep_until(timestamp + interval);
            timestamp = std::chrono::steady_clock::now();
        }

        node_.rclcpp_info("Streaming thread is stopping now");
    } // end thread process
};

Streamer::Streamer(utility::Node& node) noexcept
    : pimpl { std::make_unique<Impl>(node) } { }

Streamer::~Streamer() noexcept = default;

auto Streamer::open(const RTP_UDP& config) -> bool { return pimpl->open(config); }
auto Streamer::open(const RTSP_UDP& config) -> bool { return pimpl->open(config); }
auto Streamer::open(const RTSP_TCP& config) -> bool { return pimpl->open(config); }

auto Streamer::send(const cv::Mat& image) noexcept -> bool { return pimpl->send(image); }

auto Streamer::generate_test_image(uint8_t& hue) const noexcept -> cv::Mat {
    return pimpl->generate_test_image(hue);
}
