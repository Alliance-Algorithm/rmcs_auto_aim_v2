#include "kernel/capturer.hpp"
#include "modules/debug/visualization/stream_context.hpp"
#include "modules/debug/visualization/stream_instance.hpp"
#include "utility/image.hpp"
#include "utility/image.impl.hpp"
#include "utility/node.hpp"

#include <opencv2/highgui.hpp>
#include <rclcpp/utilities.hpp>

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

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
    using namespace rmcs::debug;

    auto check = StreamContext::check_support();
    if (!check) node.rclcpp_error("{}", check.error());

    auto configuration   = StreamSession::Target {};
    configuration.target = StreamTarget { host, port };
    configuration.type   = StreamType::RTP_JEPG;
    configuration.format = VideoFormat { w, h, hz };

    auto stream_session = StreamSession { configuration };
    stream_session.set_notifier([&](auto msg) { //
        node.rclcpp_info("[StreamSession] {}", msg);
    });
    if (auto result = stream_session.open(); !result) {
        node.rclcpp_error("{}", result.error());
        rclcpp::shutdown();
    }
    if (auto sdp = stream_session.session_description_protocol()) {
        node.rclcpp_info("\n\n\n{}\n\n", sdp.value());
    } else {
        node.rclcpp_error("{}", sdp.error());
    }
    // NOTE: End

    auto hikcamera = std::make_unique<kernel::Capturer>();
    if (!hikcamera->initialize()) { }
    hikcamera->start_working();

    auto timestamp = std::chrono::steady_clock::now();
    auto once_flag = std::once_flag {};

    while (rclcpp::ok()) {

        auto image = hikcamera->fetch();
        if (!image) continue;

        auto& current_frame = image->details().mat;
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
