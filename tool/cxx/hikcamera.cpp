#include "util/snapshot.hpp"
#include "util/terminal.hpp"
#include "utility/framerate.hpp"
#include "utility/image/recorder.hpp"
#include <module/debug/visualization/stream_session.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <deque>
#include <fstream>
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
    static constexpr auto kQueueSize = std::size_t { 120 };

    rmcs::VideoRecorder recorder;

    std::mutex mutex           = { };
    std::condition_variable cv = { };
    std::deque<cv::Mat> queue  = { };
    std::jthread worker        = { };

    std::uint64_t written = 0;
    std::uint64_t dropped = 0;

    std::atomic<bool> enabled        = false;
    std::atomic<bool> stop_requested = false;

    auto start() -> void {
        recorder.update_config({
            .directories     = { "/tmp/hikcamera_recordings/" },
            .max_duration    = std::chrono::minutes { 10 },
            .record_fps      = 60,
            .max_videos_size = 50ull * 1024 * 1024 * 1024,
        });
        if (auto result = recorder.start(); !result) {
            std::println("[recording] Failed to start: {}", result.error());
            return;
        }

        auto lock = std::lock_guard { mutex };
        written   = 0;
        dropped   = 0;
        enabled   = true;

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
                    recorder.tick(frame);
                    auto lock = std::lock_guard { mutex };
                    written += 1;
                    continue;
                }

                if (do_stop) {
                    recorder.stop();
                    continue;
                }

                if (do_exit) break;
            }
        } };
    }

    auto stop() -> void {
        {
            auto lock      = std::lock_guard { mutex };
            stop_requested = true;
            enabled        = false;
            cv.notify_one();
        }
        if (worker.joinable()) worker.join();
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

struct Streaming {
    static constexpr auto kHost = "127.0.0.1";
    static constexpr auto kPort = "5000";

    rmcs::debug::StreamSession session;
    bool enabled = false;

    auto start(int w, int h, int hz) -> void {
        auto check = rmcs::debug::StreamContext::check_support();
        if (!check) {
            std::println("[streaming] {}", check.error());
            return;
        }

        auto config   = rmcs::debug::StreamSession::Config { };
        config.target = { std::string { kHost }, std::string { kPort } };
        config.type   = rmcs::debug::StreamType::RTP_H264;
        config.format = { w, h, hz };

        session.set_notifier([](auto msg) { std::println("[streaming] {}", msg); });

        if (auto result = session.open(config); !result) {
            std::println("[streaming] Failed to open: {}", result.error());
            return;
        }

        if (auto sdp = session.session_description_protocol()) {
            const auto path = "/tmp/hikcamera_stream.sdp";
            if (auto ofs = std::ofstream { path }) {
                ofs << sdp.value();
                std::println("[streaming] SDP written to: {}", path);
            }
        }

        enabled = true;
        std::println("[streaming] Started -> {}:{}, fps={}", kHost, kPort, hz);
    }

    auto stop() -> void {
        enabled = false;
        std::println("[streaming] Stopped");
    }

    auto push(const cv::Mat& mat) -> void {
        if (enabled) session.push_frame(mat);
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
        .timeout_ms      = 2'000,
        .exposure_us     = 1'500,
        .fixed_framerate = false,
    };
    auto camera = hikcamera::Camera { };
    camera.configure(config);
    if (auto r = camera.connect()) {
        std::println("[hikcamera] Camera connect successfully");
    } else {
        std::println("[hikcamera] {}", r.error());
    }

    auto latest     = cv::Mat { };
    auto latest_mtx = std::mutex { };
    Recording recording;
    Streaming streaming;

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
                if (recording.enabled) recording.push(*mat);
                streaming.push(*mat);

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

    auto raw = rmcs::tool::util::TerminalRawMode { };
    std::println("[main] Hikcamera tool is ready");
    std::println("[main] Controls:");
    std::println("[main]   S/s  : save current frame to /tmp as PNG");
    std::println("[main]   R/r  : start / stop raw video recording");
    std::println("[main]   W/w  : start / stop RTP streaming");
    std::println("[main]   Q/q  : quit");
    std::println("[main]   Ctrl+C : quit");
    std::println("[main] Recording output: /tmp/hikcamera_recordings");
    std::println("[main] Streaming target: {}:{}", Streaming::kHost, Streaming::kPort);

    while (running.load(std::memory_order::relaxed)) {
        if (auto key = rmcs::tool::util::poll_key(std::chrono::milliseconds(100));
            key.has_value()) {
            if (*key == 'q' || *key == 'Q') {
                std::println("[main] Quit");
                running.store(false, std::memory_order::relaxed);
                break;
            }

            if (*key == 'r' || *key == 'R') {
                if (!recording.enabled) {
                    recording.start();
                    std::println("[main] Recording started");
                } else {
                    recording.stop();
                    std::println("[main] Recording stopped: written={}, dropped={}",
                        recording.written, recording.dropped);
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

            if (*key == 'w' || *key == 'W') {
                if (!streaming.enabled) {
                    auto lk = std::lock_guard { latest_mtx };
                    if (latest.empty()) {
                        std::println("[streaming] No frame available");
                    } else {
                        auto hz = static_cast<int>(framerate.fps());
                        if (hz <= 0) hz = 30;
                        streaming.start(latest.cols, latest.rows, hz);
                    }
                } else {
                    streaming.stop();
                }
                continue;
            }
        }
    }

    recording.shutdown();
    capture_thread.join();
}
