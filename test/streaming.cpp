#include "modules/debug/visualization/streamer.hpp"
#include <opencv2/opencv.hpp>
#include <rclcpp/utilities.hpp>

/// @brief
/// Block process and generate a video stream with gradient color
///
/// @note
/// target: 127.0.0.1:5000
/// width : 720
/// height: 540
/// fps   : 30
/// Just for test
/// You can use vlc to play this stream:
///
///     v=0
///     m=video 5000 RTP/AVP 26
///     c=IN IP4 192.168.3.125
///     a=rtpmap:26 JPEG/90000
///
/// Create streaming.sdp with the above content and open it with vlc
///
int main(int argc, char** argv) {
    using namespace rmcs;

    rclcpp::init(argc, argv);

    auto node = utility::Node { "test" };

    auto streamer  = std::make_unique<module::Streamer>(node);
    auto timestamp = std::chrono::steady_clock::now();

    auto config = module::Streamer::RTP_UDP {};
    config.w    = 1440;
    config.h    = 720;
    config.hz   = 165;
    config.host = "192.168.0.88";
    config.port = "5000";

    streamer->open(config);

    constexpr auto video_path = "/workspaces/RMCS/robomaster/test_hik_1.avi";
    cv::VideoCapture cap(video_path);

    if (!cap.isOpened()) {
        node.rclcpp_warn("Failed to open video file");
        return rclcpp::shutdown();
    }

    while (rclcpp::ok() && streamer->opened()) {
        cv::Mat frame;
        if (!cap.read(frame)) {
            node.rclcpp_warn("End of video reached, restarting...");
            cap.release();        // 释放旧的 VideoCapture
            cap.open(video_path); // 重新打开视频

            if (!cap.isOpened()) {
                node.rclcpp_warn("Failed to reopen video file");
                break;
            }

            continue; // 跳过当前循环，重新读取
        }

        if (!streamer->send(frame)) {
            node.rclcpp_warn("Push failed");
        }

        auto interval = std::chrono::milliseconds { 1'000 / config.hz };
        std::this_thread::sleep_until(timestamp + interval);
        timestamp = std::chrono::steady_clock::now();
    }

    return rclcpp::shutdown();
}
