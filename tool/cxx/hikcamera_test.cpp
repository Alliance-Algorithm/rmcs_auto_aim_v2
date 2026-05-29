#include <hikcamera/capturer.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <print>

using Clock     = std::chrono::steady_clock;
using TimePoint = Clock::time_point;

std::atomic<bool> running = true;

auto main() -> int {
    std::signal(SIGINT, [](int) { running = false; });

    auto config = hikcamera::Config {
        .timeout_ms  = 2'000,
        .exposure_us = 1'500,
    };
    auto camera = hikcamera::Camera {};
    camera.configure(config);

    if (auto r = camera.connect()) {
        std::println("[hikcamera_test] Camera connected");
    } else {
        std::println("[hikcamera_test] Failed to connect: {}", r.error());
        return 1;
    }

    auto frame_count = std::size_t { 0 };
    auto stat_start  = Clock::now();
    auto last_print  = Clock::now();

    std::println("[hikcamera_test] fps={:.1f}, interval={:.6f}s", 0.0, 0.0);

    while (running.load(std::memory_order::relaxed)) {
        if (auto img = camera.read_image_with_timestamp()) {
            auto now      = Clock::now();
            auto interval = now - img->timestamp;
            frame_count++;

            if (now - last_print >= std::chrono::seconds { 1 }) {
                auto elapsed_s = std::chrono::duration<double>(now - stat_start).count();
                auto fps       = elapsed_s > 0.0 ? frame_count / elapsed_s : 0.0;
                auto lag_s     = std::chrono::duration<double>(interval).count();

                std::println("[hikcamera_test] fps={:.1f}, interval={:.6f}s", fps, lag_s);

                frame_count = 0;
                stat_start  = now;
                last_print  = now;
            }
        } else {
            std::println("[hikcamera_test] Failed to read: {}", img.error());
        }
    }

    std::ignore = camera.disconnect();
    std::println("[hikcamera_test] Disconnected");
}
