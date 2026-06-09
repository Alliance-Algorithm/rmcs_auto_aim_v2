#include "recorder.hpp"
#include "utility/framerate.hpp"

#include <filesystem>
#include <format>
#include <memory>
#include <string>
#include <utility>

#include <opencv2/videoio.hpp>

using namespace rmcs;

struct VideoRecorder::Impl {
    struct Session : private cv::VideoWriter {
        using super = VideoWriter;

        FramerateCounter framerate;
        std::size_t fps = 0;

        std::chrono::steady_clock::time_point append_timestamp =
            std::chrono::steady_clock::time_point::min();
        std::chrono::steady_clock::time_point opened_timestamp =
            std::chrono::steady_clock::time_point::min();

        int cols = 0;
        int rows = 0;

        std::filesystem::path path { };
        bool remove_later = false;

        explicit Session(const std::filesystem::path& dir) {
            // 创建一个人类可读的录制文件名称并将其打开
            const auto now =
                std::chrono::floor<std::chrono::seconds>(std::chrono::system_clock::now());
            const auto our_zone  = std::chrono::locate_zone("Asia/Shanghai");
            const auto zone_time = std::chrono::zoned_time { our_zone, now };

            const auto filename  = std::format("autoaim_{:%Y-%m-%d_%H-%M-%S}.avi", zone_time);
            const auto file_path = dir / filename;

            path = file_path;
        }

        ~Session() override {
            super::release();
            if (std::filesystem::exists(path) && remove_later) {
                std::filesystem::remove(path);
            }
        }

        auto append(const cv::Mat& mat) {
            static const auto kEncode = cv::VideoWriter::fourcc('H', 'F', 'Y', 'U');
            framerate.tick();

            const auto now = std::chrono::steady_clock::now();
            if (fps == 0 && !mat.empty()) [[unlikely]] {
                if (append_timestamp == std::chrono::steady_clock::time_point::min()) {
                    append_timestamp = now;
                }
                using namespace std::chrono_literals;
                if (std::chrono::steady_clock::now() - append_timestamp > 2s) {
                    fps = framerate.fps();
                    fps = fps > 80 ? fps - 20 : fps; // 给 20 帧的冗余
                }
                return;
            }
            if (opened_timestamp == std::chrono::steady_clock::time_point::min()) {
                opened_timestamp = now;
            }

            cols = mat.cols;
            rows = mat.rows;
            if (!super::isOpened()) {
                super::open(path, kEncode, static_cast<double>(fps), { cols, rows });
                return;
            }
            using namespace std::chrono_literals;
            if (now - append_timestamp < 1000ms / fps) {
                return;
            }
            super::write(mat);
            append_timestamp = now;
        }

        auto filename() const { return path.string(); }

        auto duration() const {
            return std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - opened_timestamp);
        }

        auto set_remove_later() { remove_later = true; }

        auto set_fps(std::size_t hz) { fps = hz; }
    };
    std::unique_ptr<Session> session = nullptr;

    Config config;
    std::filesystem::path selected_directory { };

    std::optional<std::string> stop_reason = std::nullopt;

    auto stop(bool save = true) {
        if (session && !save) {
            session->set_remove_later();
        }
        if (session && !stop_reason)
            stop_reason = std::format(
                "录制已由用户停止，文件保存至 {}({})", session->filename(), session->duration());
        session = nullptr;
    }

    auto tick(const cv::Mat& mat) -> void {
        if (!session) {
            return;
        }
        // 时长过长，则自动停止录制，保存至本地
        if (session->duration() >= config.max_duration) {
            stop_reason = std::format("录制时长超过限制({})，已自动停止，文件保存至 {}",
                config.max_duration, session->filename());
            stop(true);
            return;
        }
        session->append(mat);
    }

    auto start() -> std::expected<void, std::string> {
        stop_reason.reset();

        { // 从配置中选择第一个有效的目录
            const auto& directories = config.directories;
            for (const auto& dir : directories) {
                try {
                    if (std::filesystem::exists(dir) || std::filesystem::create_directories(dir)) {
                        selected_directory = dir;
                        break;
                    }
                } catch (const std::filesystem::filesystem_error& e) {
                    continue;
                }
            }
            if (selected_directory.empty()) {
                return std::unexpected { "未能选择有效目录，请确认配置中的保存路径是否可达" };
            }
        }
        { // 判断系统空间是否充足
            constexpr auto kAllowSpaceFree = 50ull * 1024 * 1024 * 1024;

            const auto space = std::filesystem::space(selected_directory);
            if (space.available < kAllowSpaceFree) {
                const auto warn = std::format("空闲空间不足，剩余 {:.3f} GB，无法开始录制",
                    static_cast<double>(space.available) / 1024 / 1024 / 1024);
                return std::unexpected { warn };
            }
        }
        { // 查看录制保存目录下的视频是否超出限制
            auto total_size = std::uintmax_t { 0 };
            for (const auto& entry : std::filesystem::directory_iterator(selected_directory)) {
                if (entry.is_regular_file() && entry.path().extension() == ".avi") {
                    total_size += entry.file_size();
                }
            }
            if (total_size >= config.max_videos_size) {
                const auto warn = std::format("已录制文件总大小已达上限，当前大小 {:.3f} GB",
                    static_cast<double>(total_size) / 1024 / 1024 / 1024);
                return std::unexpected { warn };
            }
        }
        session = std::make_unique<Session>(selected_directory);
        if (config.record_fps != 0) {
            session->set_fps(config.record_fps);
        }

        return { };
    }

    auto status() const -> std::string {
        if (stop_reason) {
            return *stop_reason;
        }
        return session != nullptr
            ? std::format("录制中({})，当前文件为 {}", session->duration(), session->filename())
            : std::format("未开始录制");
    }
};

auto VideoRecorder::update_config(Config config) -> void { pimpl->config = std::move(config); }

auto VideoRecorder::tick(const cv::Mat& mat) -> void { pimpl->tick(mat); }

auto VideoRecorder::start() -> std::expected<void, std::string> { return pimpl->start(); }

auto VideoRecorder::stop() -> void { pimpl->stop(true); }

auto VideoRecorder::recording() const -> bool { return pimpl->session != nullptr; }

auto VideoRecorder::filename() const -> std::optional<std::string> {
    if (!pimpl->session) {
        return std::nullopt;
    }
    return pimpl->session->filename();
}

auto VideoRecorder::status() const -> std::string { return pimpl->status(); }

VideoRecorder::VideoRecorder() noexcept
    : pimpl { std::make_unique<Impl>() } { }

VideoRecorder::~VideoRecorder() noexcept = default;
