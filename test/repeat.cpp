#include "utility/repeat.hpp"

#include <gtest/gtest.h>

#include <thread>

using namespace std::chrono_literals;
using rmcs::Repeat;

// 未 start 时，tick 返回空
TEST(repeat, not_started_returns_empty) {
    auto repeat = Repeat { };
    repeat.push(Repeat::make("A", 10ms));
    EXPECT_TRUE(repeat.tick().empty());
}

// 无动作时，start 后 tick 仍返回空
TEST(repeat, empty_actions_returns_empty) {
    auto repeat = Repeat { };
    repeat.start();
    EXPECT_TRUE(repeat.tick().empty());
}

// start 之后立即 tick 应返回第一个动作（按 fill 顺序，不跳过首个）
TEST(repeat, starts_with_first_action) {
    auto repeat = Repeat { };
    repeat.push(Repeat::make("A", 50ms));
    repeat.push(Repeat::make("B", 50ms));
    repeat.push(Repeat::make("C", 50ms));

    repeat.start();
    EXPECT_EQ(repeat.tick(), "A");
}

// 按 fill 顺序正向循环：A -> B -> C -> A
TEST(repeat, cycles_in_fill_order) {
    auto repeat = Repeat { };
    repeat.push(Repeat::make("A", 30ms));
    repeat.push(Repeat::make("B", 30ms));
    repeat.push(Repeat::make("C", 30ms));

    repeat.start();
    EXPECT_EQ(repeat.tick(), "A");

    std::this_thread::sleep_for(40ms);
    EXPECT_EQ(repeat.tick(), "B");

    std::this_thread::sleep_for(30ms);
    EXPECT_EQ(repeat.tick(), "C");

    std::this_thread::sleep_for(30ms);
    EXPECT_EQ(repeat.tick(), "A");
}

// 未到时长前保持当前动作
TEST(repeat, holds_current_before_duration) {
    auto repeat = Repeat { };
    repeat.push(Repeat::make("A", 100ms));
    repeat.push(Repeat::make("B", 100ms));

    repeat.start();
    EXPECT_EQ(repeat.tick(), "A");
    std::this_thread::sleep_for(20ms);
    EXPECT_EQ(repeat.tick(), "A");
    std::this_thread::sleep_for(20ms);
    EXPECT_EQ(repeat.tick(), "A");
}

// stop 后 tick 返回空
TEST(repeat, stop_returns_empty) {
    auto repeat = Repeat { };
    repeat.push(Repeat::make("A", 50ms));

    repeat.start();
    EXPECT_EQ(repeat.tick(), "A");

    repeat.stop();
    EXPECT_TRUE(repeat.tick().empty());
}

// start 可重置进度，重新从第一个动作开始
TEST(repeat, restart_resets_to_first) {
    auto repeat = Repeat { };
    repeat.push(Repeat::make("A", 30ms));
    repeat.push(Repeat::make("B", 30ms));

    repeat.start();
    EXPECT_EQ(repeat.tick(), "A");
    std::this_thread::sleep_for(40ms);
    EXPECT_EQ(repeat.tick(), "B");

    repeat.start();
    EXPECT_EQ(repeat.tick(), "A");
}

// clear 后无动作，tick 返回空
TEST(repeat, clear_removes_actions) {
    auto repeat = Repeat { };
    repeat.push(Repeat::make("A", 30ms));
    repeat.clear();

    repeat.start();
    EXPECT_TRUE(repeat.tick().empty());
}
