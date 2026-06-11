#pragma once
#include "utility/math/linear.hpp"
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
    using Clock = std::chrono::steady_clock;

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

    /// @TODO:
    ///  融合时间戳与位姿状态的录制
    ///  预计是生成一个与视频同名的 csv 文件
    auto tick(const cv::Mat&, Clock::time_point = Clock::now()) -> void;
    auto tick(const std::string& key, const std::string& data, Clock::time_point = Clock::now())
        -> void;

    auto tick_camera_pose(const Transform& t, Clock::time_point timestamp = Clock::now()) {
        const auto [tx, ty, tz]     = t.translation;
        const auto [qx, qy, qz, qw] = t.orientation;

        const auto data = std::format("({}, {}, {}), ({}, {}, {}, {})", tx, ty, tz, qx, qy, qz, qw);

        tick("camera_pose", data, timestamp);
    }

    auto start() -> std::expected<void, std::string>;
    auto stop(bool save = true) -> void;

    auto recording() const -> bool;
    auto filename() const -> std::optional<std::string>;

    auto status() const -> std::string;
};

}
