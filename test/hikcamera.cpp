#include <gtest/gtest.h>
#include <hikcamera/capturer.hpp>

#include <csignal>
#include <future>
#include <print>

auto main() -> int {

    std::signal(SIGINT, [](auto) { std::exit(0); });

    auto camera = hikcamera::Camera {};
    if (auto result = camera.initialize()) {
        std::println("[hikcamera] Camera init successfully\n");
    } else {
        std::println("[hikcamera] {}", result.error());
    }

    std::size_t count = 0;
    while (true) {
        auto future = std::async(std::launch::async, [&camera] {
            if (auto mat = camera.read_image()) {
                std::print("[async] Read a image as cv::Mat");
                std::print(" {}x{}", mat->cols, mat->rows);
            } else {
                std::print("[async] Failed to read image: {}", mat.error());
            }
            std::println();
        });
        future.wait();
        std::println("[main] Complete image fetching {} times", count++);

        using namespace std::chrono_literals;
        std::this_thread::sleep_for(1s);
    }
}
