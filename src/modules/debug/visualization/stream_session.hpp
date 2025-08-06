#pragma once
#include <opencv2/videoio.hpp>
#include <opencv2/videoio/registry.hpp>

#include <expected>
#include <iostream>
#include <string>
#include <utility>

namespace rmcs::module {

// A simple wrapper for pipline string generation
struct Pipeline final {

    explicit Pipeline(std::string src, const auto&... pipline) noexcept
        requires(std::is_constructible_v<std::string, decltype(pipline)> && ...)
        : content(std::move(src)) {
        (content.append(std::string { " ! " } + pipline), ...);
    }

    auto append(const std::string& context) noexcept -> Pipeline& {
        content.append(" ! " + context);
        return *this;
    }

    explicit operator std::string() const noexcept { return content; }

    static auto udpsink(const std::string& host, const std::string& port, int w, int h, int fps) {
        auto pipeline = Pipeline { "appsrc" };
        {
            constexpr auto fmt_1 = "video/x-raw,format=YUY2,width={},height={},framerate={}/1";
            constexpr auto fmt_2 = "udpsink host={} port={}";

            const auto encode = std::format(fmt_1, w, h, fps);
            const auto server = std::format(fmt_2, host, port);

            pipeline.append("videoconvert");
            pipeline.append(encode);
            pipeline.append("jpegenc");
            pipeline.append("rtpjpegpay");
            pipeline.append(server);
        }
        return pipeline;
    }

    std::string content;
};

struct StreamSession {

    explicit StreamSession(const std::string& host, const std::string& port, int w, int h, int fps)
        : pipeline { Pipeline::udpsink(host, port, w, h, fps) } {

        sender = std::make_unique<cv::VideoWriter>(
            std::string { pipeline }, cv::CAP_GSTREAMER, 0, fps, cv::Size(w, h), true);

        if (!sender->isOpened()) sender.reset(nullptr);
    }

    auto opened() const noexcept { return bool { sender }; }

    static auto check_support() -> std::expected<void, std::string> {
        if (!cv::videoio_registry::getBackendName(cv::CAP_GSTREAMER).empty()) {
            return {};
        } else {
            std::cerr << "[ERROR] GStreamer backend not found in OpenCV.\n";
            std::cerr << "Please install required packages:\n"
                      << "  sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-base "
                         "gstreamer1.0-plugins-good\n";
            return std::unexpected { "Missing GStreamer support for OpenCV." };
        }
    }

    std::unique_ptr<cv::VideoWriter> sender;
    Pipeline pipeline;
};

}