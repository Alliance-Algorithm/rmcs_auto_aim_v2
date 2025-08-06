#include "modules/debug/visualization/streamer.hpp"
#include <opencv2/opencv.hpp>
#include <rclcpp/utilities.hpp>

/// @brief
/// Block process and generate a video stream with gradient color
///
/// @note
/// target: 127.0.0.1:5000
/// width : 640
/// height: 480
/// fps   : 30
/// Just for test
/// You can use vlc to play this stream:
///
///     v=0
///     m=video 5000 RTP/AVP 26
///     c=IN IP4 127.0.0.1
///     a=rtpmap:26 JPEG/90000
///
/// Create streaming.sdp with the above content and open it with vlc
///
int main(int argc, char** argv) {
    using namespace rmcs;

    rclcpp::init(argc, argv);

    auto node = utility::Node { "test" };

    auto streamer  = std::make_unique<module::Streamer>(node);
    auto hue_value = uint8_t { 0 };
    auto timestamp = std::chrono::steady_clock::now();

    auto config = module::Streamer::RTP_UDP {};
    config.w    = 720;
    config.h    = 540;
    config.hz   = 30;
    config.host = "127.0.0.1";
    config.port = "5000";

    streamer->open(config);

    while (rclcpp::ok()) {
        const auto test_image = streamer->generate_test_image(hue_value);
        if (!streamer->send(test_image)) {
            node.rclcpp_warn("Pushed failed");
        }
        auto interval = std::chrono::milliseconds { 1'000 / config.hz };
        std::this_thread::sleep_until(timestamp + interval);

        timestamp = std::chrono::steady_clock::now();
    }

    return rclcpp::shutdown();
}