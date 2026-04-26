#include <gtest/gtest.h>

#include <cstdlib>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

#include "kernel/feishu.hpp"
#include "utility/shared/context.hpp"

using namespace std::chrono_literals;
using rmcs::AutoAimState;
using rmcs::Clock;
using rmcs::ControlState;
using rmcs::kernel::Feishu;

TEST(FeishuIntegration, BidirectionalCommunication) {
    const auto pid = ::fork();
    ASSERT_NE(pid, -1);

    if (pid == 0) {
        auto feishu_child = Feishu<AutoAimState, ControlState> { };
        std::this_thread::sleep_for(50ms);

        auto deadline = Clock::now() + 500ms;
        auto ctrl     = std::optional<ControlState> { };
        while (!ctrl && Clock::now() < deadline) {
            if (feishu_child.heartbeat()) {
                ctrl = feishu_child.latest();
            } else {
                std::this_thread::sleep_for(10ms);
            }
        }
        ASSERT_TRUE(ctrl.has_value());

        auto auto_state            = AutoAimState { };
        auto_state.gimbal_takeover = true;
        auto_state.shoot_permitted = true;
        auto_state.yaw             = 1.23;

        feishu_child.send(auto_state);
        exit(0);
    }

    auto feishu_parent = Feishu<ControlState, AutoAimState> { };
    std::this_thread::sleep_for(50ms);

    feishu_parent.heartbeat();

    auto ctrl      = ControlState { };
    ctrl.timestamp = Clock::now();

    feishu_parent.send(ctrl);

    auto deadline   = Clock::now() + 500ms;
    auto auto_state = std::optional<AutoAimState> { };
    while (!auto_state && Clock::now() < deadline) {
        if (feishu_parent.heartbeat()) {
            auto_state = feishu_parent.latest();
        } else {
            std::this_thread::sleep_for(10ms);
        }
    }

    ASSERT_TRUE(auto_state.has_value());
    EXPECT_TRUE(auto_state->gimbal_takeover);
    EXPECT_TRUE(auto_state->shoot_permitted);
    EXPECT_DOUBLE_EQ(auto_state->yaw, 1.23);

    int status = 0;
    ASSERT_EQ(::waitpid(pid, &status, 0), pid);
    ASSERT_TRUE(WIFEXITED(status));
    EXPECT_EQ(WEXITSTATUS(status), 0);
}
