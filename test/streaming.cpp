#include "modules/debug/visualization/stream_context.hpp"
#include "modules/debug/visualization/stream_instance.hpp"
#include "utility/node.hpp"

#include <hikcamera/image_capturer.hpp>
#include <opencv2/highgui.hpp>
#include <rclcpp/utilities.hpp>

#include <chrono>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

int main(int argc, char** argv) {
    using namespace rmcs;

    rclcpp::init(argc, argv);

    auto node = utility::Node { "streaming_test" };

    constexpr auto host = std::string_view { "127.0.0.1" };
    constexpr auto port = std::string_view { "5000" };
    constexpr auto hz   = int { 80 };
    constexpr auto w    = int { 1440 };
    constexpr auto h    = int { 1080 };

    // NOTE: Stream Session
    using debug::StreamSession;
    auto stream_session = StreamSession {
        StreamSession::StreamType::RTP_JEPG,
        StreamSession::StreamTarget { host, port },
        StreamSession::VideoFormat { w, h, hz },
    };
    stream_session.set_notifier([&](auto msg) { node.rclcpp_info("StreamSession: {}", msg); });
    auto result = stream_session.open();
    if (!result) {
        node.rclcpp_error("{}", result.error());
    }

    auto sdp = stream_session.session_description_protocol();
    if (sdp) {
        node.rclcpp_info("Sdp:\n{}", sdp.value());
    } else {
        node.rclcpp_error("{}", sdp.error());
    }
    // NOTE: End

    auto camera  = std::unique_ptr<hikcamera::ImageCapturer> {};
    auto profile = hikcamera::ImageCapturer::CameraProfile {};

    using namespace std::chrono_literals;
    profile.exposure_time = 2ms;
    profile.invert_image  = false;

    const auto try_make_hikcamera = [&] {
        camera.reset();
        while (rclcpp::ok() && !camera) {
            try {
                camera = std::make_unique<hikcamera::ImageCapturer>(profile);
                camera->set_frame_rate_inner_trigger_mode(hz);
            } catch (const std::runtime_error& e) {

                node.rclcpp_error("Hikcamera: {}", e.what());
                std::this_thread::yield();

                using namespace std::chrono_literals;
                std::this_thread::sleep_for(2s);
            }
        }
    };
    try_make_hikcamera();

    auto check = debug::StreamContext::check_support();
    if (!check) node.rclcpp_error("{}", check.error());

    auto timestamp = std::chrono::steady_clock::now();
    auto once_flag = std::once_flag {};

    while (rclcpp::ok()) {

        auto current_frame = cv::Mat {};
        try {
            using namespace std::chrono_literals;
            current_frame = camera->read(200ms);
        } catch (const std::runtime_error& error) {
            node.rclcpp_error("Error while capturing: {}", error.what());
            try_make_hikcamera();
        }

        std::call_once(once_flag, [&] {
            const auto frame_w = current_frame.cols;
            const auto frame_h = current_frame.rows;

            if (frame_w != w || frame_h != h) {
                node.rclcpp_error("Given size is not fit with {}x{}", frame_w, frame_h);
                rclcpp::shutdown();
            }
        });

        // NOTE: Stream Session
        if (!stream_session.push_frame(current_frame)) {
            node.rclcpp_warn("Frame was pushed failed");
        }
        // NOTE: End

        const auto interval   = std::chrono::steady_clock::now() - timestamp;
        const auto frame_cost = std::chrono::duration<double> { interval };
        const auto frame_rate = 1. / frame_cost.count();

        {
            static auto info_timestamp = std::chrono::steady_clock::now();

            const auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - info_timestamp)
                >= std::chrono::seconds(3)) {
                node.rclcpp_info("Frame Rate: {}", frame_rate);
                info_timestamp = now;
            }
        }
        timestamp = std::chrono::steady_clock::now();
    }

    return rclcpp::shutdown();
}
