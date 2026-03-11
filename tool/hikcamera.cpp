#include "util/snapshot.hpp"
#include "util/terminal.hpp"
#include <hikcamera/capturer.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <mutex>
#include <opencv2/imgcodecs.hpp>
#include <print>
#include <thread>

std::atomic<bool> running = true;

auto main() -> int {
    std::signal(SIGINT, [](auto) { running = false; });

    auto config = hikcamera::Config {
        .timeout_ms  = 2'000,
        .exposure_us = 1'500,
        // ...
    };
    auto camera = hikcamera::Camera { };
    camera.configure(config);

    if (auto result = camera.connect()) {
        std::println("[hikcamera] Camera connect successfully");
    } else {
        std::println("[hikcamera] {}", result.error());
    }

    auto latest_image = cv::Mat { };
    auto image_mutex  = std::mutex { };

    std::jthread capture_thread([&camera, &latest_image, &image_mutex] {
        std::size_t count = 0;
        while (running.load(std::memory_order::relaxed)) {
            if (!camera.connected()) {
                if (auto ret = camera.connect(); !ret) {
                    std::println("[capture] Connected failed: {}", ret.error());
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    continue;
                }
            }

            if (auto mat = camera.read_image()) {
                std::lock_guard<std::mutex> lock(image_mutex);
                latest_image = mat->clone();
                std::println("[capture] Read image {} ({}x{})", count++, mat->cols, mat->rows);
            } else {
                std::println("[capture] Failed to read image: {}", mat.error());
            }
        }
    });

    auto raw_mode = rmcs::tool::util::TerminalRawMode { };
    std::println("[main] Press S to save current frame, Q to exit, Ctrl+C to exit");

    while (running.load(std::memory_order::relaxed)) {
        if (auto key = rmcs::tool::util::poll_key(std::chrono::milliseconds(100));
            key.has_value()) {
            if (*key == 'q' || *key == 'Q') {
                std::println("[main] Quit requested by keyboard");
                running.store(false, std::memory_order::relaxed);
                break;
            }

            if (*key != 's' && *key != 'S') {
                continue;
            }

            auto snapshot = cv::Mat { };
            {
                std::lock_guard<std::mutex> lock(image_mutex);
                if (!latest_image.empty()) {
                    snapshot = latest_image.clone();
                }
            }

            if (snapshot.empty()) {
                std::println("[main] No image available yet, skip saving");
                continue;
            }

            const auto path = rmcs::tool::util::build_snapshot_path();
            if (cv::imwrite(path, snapshot)) {
                std::println("[main] Saved image to {}", path);
            } else {
                std::println("[main] Failed to save image to {}", path);
            }
        }
    }
    capture_thread.join();
}
