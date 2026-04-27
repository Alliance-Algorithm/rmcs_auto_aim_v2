#include "recorder.hpp"

#include <filesystem>
#include <memory>
#include <ranges>
#include <string>
#include <system_error>
#include <vector>

#include <opencv2/videoio.hpp>

using namespace rmcs;

struct ImageRecorder::Impl {
    using Clock     = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    std::string location = "/tmp/auto_aim_recordings";

    std::size_t framerate   = 30;
    bool auto_save          = true;
    std::size_t max_history = 5;

    std::chrono::seconds max_duration { 60 };
    std::chrono::seconds min_duration { 1 };

    bool is_recording = false;

    TimePoint recording_start { };

    int frame_count = 0;
    int width       = 0;
    int height      = 0;

    std::string current_file_path = { };
    std::string last_saved_path   = { };

    std::unique_ptr<cv::VideoWriter> writer = { };

    ~Impl() noexcept {
        if (auto_save) {
            stop();
        } else {
            finalize_recording(false);
        }
    }

    auto generate_output_path() const -> std::string {
        const auto now = std::chrono::system_clock::now();
        const auto timestamp =
            std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

        return location + "/" + std::to_string(timestamp) + ".avi";
    }

    auto start_recording(const cv::Mat& mat) -> void {
        if (mat.empty()) {
            return;
        }

        last_saved_path = { };

        std::error_code error = { };
        std::filesystem::create_directories(location, error);
        if (error) {
            return;
        }

        width  = mat.cols;
        height = mat.rows;

        current_file_path = generate_output_path();
        writer            = std::make_unique<cv::VideoWriter>();

        if (!writer->open(current_file_path, cv::VideoWriter::fourcc('H', 'F', 'Y', 'U'),
                static_cast<double>(framerate), cv::Size { width, height })) {
            writer.reset();
            current_file_path = { };
            width             = 0;
            height            = 0;
            frame_count       = 0;
            is_recording      = false;
            return;
        }

        const auto now  = Clock::now();
        recording_start = now;
        frame_count     = 0;
        is_recording    = true;
    }

    auto is_valid_duration() const noexcept -> bool {
        if (!is_recording && recording_start == TimePoint { }) {
            return false;
        }

        const auto duration = Clock::now() - recording_start;
        return duration >= min_duration && duration <= max_duration;
    }

    auto cleanup_old_recordings() noexcept -> void {
        if (max_history == 0) {
            return;
        }

        std::error_code error = { };
        if (!std::filesystem::exists(location, error) || error) {
            return;
        }

        auto recordings = std::vector<std::filesystem::directory_entry> { };
        for (const auto& entry : std::filesystem::directory_iterator { location, error }) {
            if (error) {
                return;
            }

            if (entry.is_regular_file(error) && !error && entry.path().extension() == ".avi") {
                recordings.push_back(entry);
            }
            error.clear();
        }

        if (recordings.size() <= max_history) {
            return;
        }

        std::ranges::sort(recordings, [](const auto& lhs, const auto& rhs) {
            std::error_code lhs_error = { };
            std::error_code rhs_error = { };
            const auto lhs_time       = std::filesystem::last_write_time(lhs, lhs_error);
            const auto rhs_time       = std::filesystem::last_write_time(rhs, rhs_error);

            if (lhs_error || rhs_error) {
                return lhs.path().filename().string() > rhs.path().filename().string();
            }
            return lhs_time > rhs_time;
        });

        for (const auto& entry : recordings | std::views::drop(max_history)) {
            std::filesystem::remove(entry.path(), error);
            error.clear();
        }
    }

    auto finalize_recording(bool keep) noexcept -> void {
        if (writer) {
            writer->release();
            writer.reset();
        }

        if (!current_file_path.empty()) {
            std::error_code error = { };
            if (keep) {
                last_saved_path = current_file_path;
                cleanup_old_recordings();
            } else {
                std::filesystem::remove(current_file_path, error);
                last_saved_path = { };
            }
        }

        current_file_path = { };
        recording_start   = TimePoint { };
        frame_count       = 0;
        width             = 0;
        height            = 0;
        is_recording      = false;
    }

    auto write_frame(const cv::Mat& mat) -> void {
        if (mat.empty()) {
            return;
        }

        if (!is_recording) {
            start_recording(mat);
        }

        if (!writer || !is_recording) {
            return;
        }

        if (mat.cols != width || mat.rows != height) {
            finalize_recording(false);
            start_recording(mat);
            if (!writer || !is_recording) {
                return;
            }
        }

        writer->write(mat.clone());
        frame_count += 1;
    }

    auto stop() -> void {
        if (!writer || !is_recording) {
            return;
        }

        const auto keep = auto_save && is_valid_duration() && frame_count > 0;
        finalize_recording(keep);
    }
};

auto ImageRecorder::set_saving_location(const std::string& path) -> void { pimpl->location = path; }

auto ImageRecorder::set_framerate(std::size_t rate) -> void { pimpl->framerate = rate; }

auto ImageRecorder::set_auto_save(bool enabled) -> void { pimpl->auto_save = enabled; }

auto ImageRecorder::set_max_history_count(std::size_t count) -> void { pimpl->max_history = count; }

auto ImageRecorder::set_max_recording_duration(std::chrono::seconds duration) -> void {
    pimpl->max_duration = duration;
}

auto ImageRecorder::set_min_recording_duration(std::chrono::seconds duration) -> void {
    pimpl->min_duration = duration;
}

auto ImageRecorder::write_frame(const cv::Mat& mat) -> void { pimpl->write_frame(mat); }

auto ImageRecorder::stop() -> void { pimpl->stop(); }

auto ImageRecorder::recording() const -> bool { return pimpl->is_recording; }

auto ImageRecorder::current_file_path() const -> std::string { return pimpl->current_file_path; }

auto ImageRecorder::last_saved_path() const -> std::string { return pimpl->last_saved_path; }

ImageRecorder::ImageRecorder() noexcept
    : pimpl { std::make_unique<Impl>() } { }

ImageRecorder::~ImageRecorder() noexcept = default;
