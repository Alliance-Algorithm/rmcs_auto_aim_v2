#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/videoio/registry.hpp>

#include <expected>
#include <iostream>
#include <string>
#include <utility>

namespace rmcs::module {

struct StreamSession {

    static constexpr auto template_rtp = [] {
        return "appsrc "
               "! videoconvert "
               "! video/x-raw,format=YUY2,width={},height={},framerate={}/1 "
               "! jpegenc "
               "! rtpjpegpay "
               "! udpsink host={} port={}";
    };
    static constexpr auto template_rtsp = [] {
        return "appsrc "
               "! videoconvert "
               "! video/x-raw,format=YUY2,width={},height={},framerate={}/1 "
               "! x264enc tune=zerolatency bitrate=600 speed-preset=ultrafast "
               "! rtph264pay config-interval=1 pt=96 "
               "! udpsink host={} port={}";
    };

    template <auto fmt>
    auto open(int w, int h, int hz, std::string host, std::string port) noexcept
        -> std::expected<void, std::string> {

        this->pipeline = std::format(fmt(), w, h, hz, host, port);
        this->host     = std::move(host);
        this->port     = std::move(port);
        this->w        = w;
        this->h        = h;
        this->hz       = hz;

        this->sender = std::make_unique<cv::VideoWriter>(
            pipeline, cv::CAP_GSTREAMER, 0, hz, cv::Size(w, h), true);

        if (sender->isOpened() == false) {
            sender.reset(nullptr);
            return std::unexpected {
                "[ERROR] Unable to open pipeline.\n"
                "Please install required packages or check your pipeline config\n"
                "  sudo apt install "
                "gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good\n",
            };
        }
        return {};
    }

    auto open_rtp(int w, int h, int hz, std::string host, std::string port) noexcept
        -> std::expected<void, std::string> {
        return open<template_rtp>(w, h, hz, std::move(host), std::move(port));
    }
    auto open_rtsp(int w, int h, int hz, std::string host, std::string port) noexcept
        -> std::expected<void, std::string> {
        return open<template_rtsp>(w, h, hz, std::move(host), std::move(port));
    }

    auto opened() const noexcept { return bool { sender }; }

    auto send(const cv::Mat& content) const noexcept {
        if (sender) sender->write(content);
    }

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
    std::string pipeline;

    int w, h, hz;
    std::string host, port;
};

}
