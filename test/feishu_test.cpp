#include <gtest/gtest.h>

#include <cstdlib>
#include <optional>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

#include "kernel/feishu.hpp"

using namespace std::chrono_literals;
using rmcs::kernel::Feishu;
using rmcs::util::AutoAimSide;
using rmcs::util::AutoAimState;
using rmcs::util::Clock;
using rmcs::util::ControlSide;
using rmcs::util::ControlState;

// Illegal operation should not be supported (compile-time check).
static_assert(!rmcs::util::ShmRoleSelector<AutoAimSide, ControlState>::is_sender);
static_assert(!rmcs::util::ShmRoleSelector<ControlSide, AutoAimState>::is_sender);

TEST(FeishuIntegration, BidirectionalCommunication) {
    const auto pid = ::fork();
    ASSERT_NE(pid, -1);

    if (pid == 0) {
        auto feishu_child = Feishu<AutoAimSide> {};
        std::this_thread::sleep_for(50ms); // ensure shm is ready

        auto deadline = Clock::now() + 10ms;
        auto ctrl     = std::optional<ControlState> {};
        while (!ctrl && Clock::now() < deadline) {
            ctrl = feishu_child.fetch<ControlState>();
            if (!ctrl) std::this_thread::sleep_for(10ms);
        }
        ASSERT_TRUE(ctrl.has_value());

        auto auto_state             = AutoAimState {};
        auto_state.should_control   = true;
        auto_state.should_shoot     = true;
        auto_state.target_posture.x = 1.23;

        ASSERT_TRUE(feishu_child.commit<AutoAimState>(auto_state));
        std::_Exit(0);
    }

    auto feishu_parent = Feishu<ControlSide> {};
    std::this_thread::sleep_for(50ms); // ensure shm is ready

    auto ctrl         = ControlState {};
    ctrl.bullet_speed = 42.0;
    ctrl.timestamp    = Clock::now();

    ASSERT_TRUE(feishu_parent.commit<ControlState>(ctrl));

    auto deadline   = Clock::now() + 10ms;
    auto auto_state = std::optional<AutoAimState> {};
    while (!auto_state && Clock::now() < deadline) {
        auto_state = feishu_parent.fetch<AutoAimState>();
        if (!auto_state) std::this_thread::sleep_for(10ms);
    }

    ASSERT_TRUE(auto_state.has_value());
    EXPECT_TRUE(auto_state->should_control);
    EXPECT_TRUE(auto_state->should_shoot);
    EXPECT_DOUBLE_EQ(auto_state->target_posture.x, 1.23);

    int status = 0;
    ASSERT_EQ(::waitpid(pid, &status, 0), pid);
    ASSERT_TRUE(WIFEXITED(status));
    EXPECT_EQ(WEXITSTATUS(status), 0);
}
