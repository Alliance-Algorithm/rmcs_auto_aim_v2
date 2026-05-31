#include "module/debug/framerate.hpp"
#include "util/snapshot.hpp"
#include "util/terminal.hpp"
#include "utility/image/recorder.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <deque>
#include <mutex>
#include <print>
#include <string>
#include <thread>

#include <hikcamera/capturer.hpp>
#include <opencv2/imgcodecs.hpp>

using Clock     = std::chrono::steady_clock;
using TimePoint = Clock::time_point;

namespace {

struct Recording {
    static constexpr auto kQueueSize    = std::size_t { 120 };
    static constexpr auto kStatInterval = std::chrono::seconds { 2 };

    rmcs::ImageRecorder recorder;

    std::mutex mutex           = { };
    std::condition_variable cv = { };
    std::deque<cv::Mat> queue  = { };
    std::jthread worker        = { };

    std::uint64_t written = 0;
    std::uint64_t dropped = 0;

    TimePoint started_at                      = { };
    std::chrono::nanoseconds total_write_time = { };
    TimePoint stat_last                       = { };
    std::uint64_t stat_written                = 0;
    std::chrono::nanoseconds stat_write_time  = { };

    bool enabled        = false;
    bool stop_requested = false;
    bool stop_pending   = false;

    auto start(std::size_t framerate) -> void {
        recorder.set_saving_location("/tmp/hikcamera_recordings");
        recorder.set_framerate(framerate);
        recorder.set_auto_save(true);
        recorder.set_max_history_count(10);
        recorder.set_min_recording_duration(std::chrono::seconds { 0 });
        recorder.set_max_recording_duration(std::chrono::hours { 24 });

        auto lock        = std::lock_guard { mutex };
        started_at       = Clock::now();
        written          = 0;
        dropped          = 0;
        total_write_time = std::chrono::nanoseconds { 0 };
        stat_last        = started_at;
        stat_written     = 0;
        stat_write_time  = std::chrono::nanoseconds { 0 };
        enabled          = true;
        stop_pending     = false;

        worker = std::jthread { [this] {
            for (;;) {
                auto frame   = cv::Mat { };
                auto do_stop = false;
                auto do_exit = false;

                {
                    auto lock = std::unique_lock { mutex };
                    cv.wait(lock, [this] { return !enabled || !queue.empty() || stop_requested; });

                    if (!queue.empty()) {
                        frame = std::move(queue.front());
                        queue.pop_front();
                    } else if (stop_requested) {
                        stop_requested = false;
                        do_stop        = true;
                    } else if (!enabled) {
                        do_exit = true;
                    }
                }

                if (!frame.empty()) {
                    const auto t_before = Clock::now();
                    recorder.write_frame(frame);
                    const auto t_after = Clock::now();

                    auto lock = std::lock_guard { mutex };
                    written += 1;
                    stat_written += 1;
                    const auto elapsed = t_after - t_before;
                    total_write_time += elapsed;
                    stat_write_time += elapsed;

                    if (t_after - stat_last >= kStatInterval) {
                        const auto fps = static_cast<double>(stat_written)
                            / std::chrono::duration<double>(t_after - stat_last).count();
                        const auto avg_ms = stat_written > 0
                            ? std::chrono::duration_cast<std::chrono::milliseconds>(stat_write_time)
                                    .count()
                                / stat_written
                            : 0;

                        std::println("[recording] write fps={:.1f}, avg write latency={}ms, q={}",
                            fps, avg_ms, queue.size());
                        stat_last       = t_after;
                        stat_written    = 0;
                        stat_write_time = std::chrono::nanoseconds { 0 };
                    }
                    continue;
                }

                if (do_stop) {
                    recorder.stop();
                    auto lock    = std::lock_guard { mutex };
                    stop_pending = false;
                    cv.notify_all();
                    continue;
                }

                if (do_exit) break;
            }
        } };
    }

    auto request_stop() -> void {
        auto lock      = std::lock_guard { mutex };
        stop_requested = true;
        stop_pending   = true;
        cv.notify_one();
    }

    auto wait_stop(std::string& saved_path, std::uint64_t& out_written, std::uint64_t& out_dropped,
        std::chrono::nanoseconds& out_duration) -> void {
        auto lock = std::unique_lock { mutex };
        cv.wait(lock, [this] { return !stop_pending; });
        saved_path   = recorder.last_saved_path();
        out_written  = written;
        out_dropped  = dropped;
        out_duration = Clock::now() - started_at;
    }

    auto shutdown() -> void {
        enabled = false;
        cv.notify_one();
        if (worker.joinable()) worker.join();
    }

    auto push(const cv::Mat& mat) -> void {
        auto lock = std::lock_guard { mutex };
        if (queue.size() >= kQueueSize) {
            queue.pop_front();
            dropped += 1;
        }
        queue.push_back(mat.clone());
        cv.notify_one();
    }
};

auto save_snapshot(const cv::Mat& src) -> void {
    const auto path = rmcs::tool::util::build_snapshot_path();
    if (cv::imwrite(path, src)) {
        std::println("[main] Saved image to {}", path);
    } else {
        std::println("[main] Failed to save image to {}", path);
    }
}

} // namespace

std::atomic<bool> running = true;

auto main() -> int {
    std::signal(SIGINT, [](auto) { running = false; });

    auto framerate = rmcs::FramerateCounter { };
    framerate.set_interval(std::chrono::seconds { 2 });

    auto config = hikcamera::Config {
        .timeout_ms  = 2'000,
        .exposure_us = 1'500,
    };
    auto camera = hikcamera::Camera { };
    camera.configure(config);
    if (auto r = camera.connect()) {
        std::println("[hikcamera] Camera connect successfully");
    } else {
        std::println("[hikcamera] {}", r.error());
    }

    auto const recording_fps = static_cast<std::size_t>(
        std::max<long long>(1, static_cast<long long>(std::llround(config.framerate))));

    auto latest     = cv::Mat { };
    auto latest_mtx = std::mutex { };
    Recording recording;
    auto rec_enabled = std::atomic<bool> { false };

    auto capture_thread = std::jthread { [&] {
        while (running.load(std::memory_order::relaxed)) {
            if (!camera.connected()) {
                if (auto r = camera.connect(); !r) {
                    std::println("[capture] Connect failed: {}", r.error());
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    continue;
                }
            }
            if (auto mat = camera.read_image()) {
                if (rec_enabled.load(std::memory_order::relaxed)) recording.push(*mat);

                {
                    auto lk = std::lock_guard { latest_mtx };
                    latest  = mat->clone();
                }
                if (framerate.tick())
                    std::println("[capture] capture framerate {}hz", framerate.fps());
            } else {
                std::println("[capture] Failed to read image: {}", mat.error());
            }
        }
    } };

    auto _raw = rmcs::tool::util::TerminalRawMode { };
    std::println("[main] Hikcamera tool is ready");
    std::println("[main] Controls:");
    std::println("[main]   S/s  : save current frame to /tmp as PNG");
    std::println("[main]   R/r  : start / stop raw video recording");
    std::println("[main]   Q/q  : quit");
    std::println("[main]   Ctrl+C : quit");
    std::println("[main] Recording output: /tmp/hikcamera_recordings");
    std::println("[main] Recording framerate: {} fps", recording_fps);

    while (running.load(std::memory_order::relaxed)) {
        if (auto key = rmcs::tool::util::poll_key(std::chrono::milliseconds(100));
            key.has_value()) {
            if (*key == 'q' || *key == 'Q') {
                std::println("[main] Quit");
                running.store(false, std::memory_order::relaxed);
                break;
            }

            if (*key == 'r' || *key == 'R') {
                if (!rec_enabled.load(std::memory_order::relaxed)) {
                    rec_enabled.store(true, std::memory_order::relaxed);
                    recording.start(recording_fps);
                    std::println("[main] Recording started");
                } else {
                    if (recording.stop_pending) {
                        std::println("[main] Stop in progress, please wait");
                        continue;
                    }
                    rec_enabled.store(false, std::memory_order::relaxed);
                    recording.request_stop();
                    std::println("[main] Stopping...");

                    auto saved = std::string { };
                    auto w = std::uint64_t { }, d = std::uint64_t { };
                    auto dur = std::chrono::nanoseconds { };
                    recording.wait_stop(saved, w, d, dur);

                    if (saved.empty()) {
                        std::println("[main] Recording stopped, no file saved");
                    } else {
                        std::println("[main] Saved: {}", saved);
                        std::println("[main] Stats: {}ms, written={}, dropped={}",
                            std::chrono::duration_cast<std::chrono::milliseconds>(dur).count(), w,
                            d);
                    }
                }
                continue;
            }

            if (*key == 's' || *key == 'S') {
                auto snap = cv::Mat { };
                {
                    auto lk = std::lock_guard { latest_mtx };
                    if (!latest.empty()) snap = latest.clone();
                }
                if (snap.empty()) {
                    std::println("[main] No image available");
                } else {
                    save_snapshot(snap);
                }
                continue;
            }
        }
    }

    rec_enabled.store(false, std::memory_order::relaxed);
    recording.shutdown();
    capture_thread.join();
}
