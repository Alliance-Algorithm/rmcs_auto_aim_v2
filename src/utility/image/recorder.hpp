#pragma once
#include "utility/pimpl.hpp"

#include <chrono>
#include <expected>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>

namespace rmcs {

class VideoRecorder {
    RMCS_PIMPL_DEFINITION(VideoRecorder)
public:
    struct Config {
        std::vector<std::string> directories {
            "/home/root/autoaim/",
            "/home/ubuntu/autoaim/",
            "/tmp/autoaim/",
        };
        std::chrono::seconds max_duration { 60 };
        std::size_t record_fps = 0;

        std::uintmax_t max_videos_size = 30ull * 1024 * 1024 * 1024; // 30 GB
    };
    auto update_config(Config) -> void;

    auto tick(const cv::Mat&) -> void;

    auto start() -> std::expected<void, std::string>;
    auto stop() -> void;

    auto recording() const -> bool;
    auto filename() const -> std::optional<std::string>;
};

}
