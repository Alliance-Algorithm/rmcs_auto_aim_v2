#include <gtest/gtest.h>
#include <hikcamera/capturer.hpp>
#include <print>

GTEST_TEST(hikcamera, build) {
    auto camera = hikcamera::Camera {};
    if (auto result = camera.initialize()) {
        std::println("Camera init successfully\n{}", *result);
    } else {
        std::println("{}", result.error());
    }
}
