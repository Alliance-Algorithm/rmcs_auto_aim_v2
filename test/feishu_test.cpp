#include <gtest/gtest.h>

#include <cstdlib>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

#include "kernel/feishu.hpp"
#include "utility/shared/context.hpp"

using namespace std::chrono_literals;
using namespace rmcs;
using namespace rmcs::kernel;

TEST(FeishuIntegration, BidirectionalCommunication) {
    const auto pid = ::fork();
    ASSERT_NE(pid, -1);

    if (pid == 0) {
        auto feishu_child = Feishu<AutoAimState, SystemContext> { };
        std::this_thread::sleep_for(50ms);

        auto deadline = Clock::now() + 500ms;
        auto ctrl     = std::optional<SystemContext> { };
        while (!ctrl && Clock::now() < deadline) {
            if (feishu_child.heartbeat()) {
                ctrl = feishu_child.latest();
            } else {
                std::this_thread::sleep_for(10ms);
            }
        }
        ASSERT_TRUE(ctrl.has_value());

        auto auto_state           = AutoAimState { };
        auto_state.should_control = true;
        auto_state.should_shoot   = true;
        auto_state.yaw            = 1.23;

        feishu_child.send(auto_state);
        exit(0);
    }

    auto feishu_parent = Feishu<SystemContext, AutoAimState> { };
    std::this_thread::sleep_for(50ms);

    feishu_parent.heartbeat();

    auto ctrl      = SystemContext { };
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
    EXPECT_TRUE(auto_state->should_control);
    EXPECT_TRUE(auto_state->should_shoot);
    EXPECT_DOUBLE_EQ(auto_state->yaw, 1.23);

    int status = 0;
    ASSERT_EQ(::waitpid(pid, &status, 0), pid);
    ASSERT_TRUE(WIFEXITED(status));
    EXPECT_EQ(WEXITSTATUS(status), 0);
}
