#include "module/capturer/hikcamera.hpp"
#include "module/debug/framerate.hpp"
#include "module/debug/visualization/stream_session.hpp"
#include "util/snapshot.hpp"
#include "util/terminal.hpp"
#include "utility/image/image.details.hpp"
#include "utility/rclcpp/node.hpp"

#include <rclcpp/utilities.hpp>

#include <opencv2/imgcodecs.hpp>

#include <charconv>
#include <chrono>
#include <csignal>
#include <fstream>
#include <memory>
#include <mutex>
#include <string_view>
#include <system_error>
#include <thread>

int main(int argc, char** argv) {
    using namespace rmcs;

    constexpr auto default_exposure_us = int { 3'000 };

    rclcpp::init(argc, argv);
    std::signal(SIGINT, [](auto) { rclcpp::shutdown(); });

    auto node = util::RclcppNode { "streaming_test" };

    auto exposure_us = default_exposure_us;
    if (argc >= 2) {
        const auto arg = std::string_view { argv[1] };
        auto parsed    = int { };
        const auto ret = std::from_chars(arg.data(), arg.data() + arg.size(), parsed);
        if (ret.ec == std::errc { } && ret.ptr == arg.data() + arg.size() && parsed > 0) {
            exposure_us = parsed;
        } else {
            node.warn("Invalid exposure_us '{}', fallback to {}", arg, default_exposure_us);
        }
    }
    node.info("Using exposure_us={}", exposure_us);

    constexpr auto host = "127.0.0.1";
    constexpr auto port = "5000";
    constexpr auto hz   = int { 80 };
    constexpr auto w    = int { 1440 };
    constexpr auto h    = int { 1080 };

    // NOTE: Stream Session
    using namespace rmcs::debug;

    auto check = StreamContext::check_support();
    if (!check) node.error("{}", check.error());

    auto config   = StreamSession::Config { };
    config.target = StreamTarget { host, port };
    config.type   = StreamType::RTP_JEPG;
    config.format = VideoFormat { w, h, hz };

    auto stream_session = StreamSession { };
    stream_session.set_notifier([&](auto msg) { //
        node.info("[StreamSession] {}", msg);
    });
    if (auto result = stream_session.open(config); !result) {
        node.error("{}", result.error());
        rclcpp::shutdown();
    }
    if (auto sdp = stream_session.session_description_protocol()) {
        const auto output_location = "/tmp/auto_aim.sdp";
        if (auto ofstream = std::ofstream { output_location }) {
            ofstream << sdp.value();
            node.info("SDP written to: {}", output_location);
        } else {
            node.error("Failed to write SDP: {}", output_location);
            rclcpp::shutdown();
        }
    } else {
        node.error("{}", sdp.error());
    }
    // NOTE: End

    auto hikcamera = std::make_unique<cap::Hikcamera>();
    hikcamera->configure({
        .exposure_us = static_cast<float>(exposure_us),
    });

    if (auto ret = hikcamera->connect(); !ret) {
        node.error("Failed to init camera: {}", ret.error());
    } else {
        node.info("Successfully initialize camera");
    }

    auto once_flag = std::once_flag { };
    auto framerate = FramerateCounter { };
    auto raw_mode  = rmcs::tool::util::TerminalRawMode { };
    auto save_next = false;

    node.info("Press S to save frame, Q to exit");

    while (rclcpp::ok()) {
        if (auto key = rmcs::tool::util::poll_key(std::chrono::milliseconds(0)); key.has_value()) {
            if (*key == 'q' || *key == 'Q') {
                node.info("Quit requested by keyboard");
                rclcpp::shutdown();
                break;
            }
            if (*key == 's' || *key == 'S') {
                save_next = true;
            }
        }

        auto image = hikcamera->wait_image();
        if (!image.has_value()) {
            node.error("Failed to read image: {}", image.error());

            if (auto ret = hikcamera->connect(); !ret) {
                node.error("Failed to init camera: {}", ret.error());
                std::this_thread::sleep_for(std::chrono::seconds { 2 });
            } else {
                node.info("Successfully initialize camera");
            }

            continue;
        }

        auto& current_frame = image.value();
        std::call_once(once_flag, [&] {
            const auto frame_w = current_frame->details().get_cols();
            const auto frame_h = current_frame->details().get_rows();

            if (frame_w != w || frame_h != h) {
                node.error("Given size {}x{} != target[{}x{}]", frame_w, frame_h, w, h);
                rclcpp::shutdown();
            } else {
                node.info("First frame was received");
            }
        });
        if (framerate.tick()) {
            node.info("Hz: {}", framerate.fps());
        }

        // NOTE: Stream Session
        if (!stream_session.push_frame(current_frame->details().mat)) {
            node.warn("Frame was pushed failed");
        }
        // NOTE: End

        if (save_next) {
            const auto path = rmcs::tool::util::build_snapshot_path();
            if (cv::imwrite(path, current_frame->details().mat)) {
                node.info("Saved image: {}", path);
            } else {
                node.error("Failed to save image: {}", path);
            }
            save_next = false;
        }
    }

    return rclcpp::shutdown();
}
