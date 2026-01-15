#include <gtest/gtest.h>

#include "module/debug/action_throttler.hpp"

using namespace std::chrono_literals;
using rmcs::util::ActionThrottler;

TEST(ActionThrottler, DispatchByIntervalAndQuota) {
    ActionThrottler throttler { 10ms, 2 }; // 10ms 节拍，配额 2 次
    throttler.register_action("foo");

    int count = 0;
    // 第一次 tick -> 执行
    EXPECT_TRUE(throttler.dispatch("foo", [&] { ++count; }));
    // 同一节拍内第二次调用 metronome 未到达，直接 false
    EXPECT_FALSE(throttler.dispatch("foo", [&] { ++count; }));

    // 等待到下一个节拍
    std::this_thread::sleep_for(12ms);
    EXPECT_TRUE(throttler.dispatch("foo", [&] { ++count; }));

    // 配额用完，再到下个节拍也不会执行
    std::this_thread::sleep_for(12ms);
    EXPECT_FALSE(throttler.dispatch("foo", [&] { ++count; }));

    EXPECT_EQ(count, 2);
}

TEST(ActionThrottler, ResetRestoreQuota) {
    ActionThrottler throttler { 1ms, 1 };
    throttler.register_action("bar");

    int count = 0;
    EXPECT_TRUE(throttler.dispatch("bar", [&] { ++count; }));
    // 配额耗尽
    std::this_thread::sleep_for(2ms);
    EXPECT_FALSE(throttler.dispatch("bar", [&] { ++count; }));

    throttler.reset("bar");
    std::this_thread::sleep_for(2ms);
    EXPECT_TRUE(throttler.dispatch("bar", [&] { ++count; }));
    EXPECT_EQ(count, 2);
}

