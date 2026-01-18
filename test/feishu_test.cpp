#include <gtest/gtest.h>

#include <cstdlib>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

#include "kernel/feishu.hpp"

using namespace std::chrono_literals;
using rmcs::kernel::Feishu;
using rmcs::kernel::RuntimeRole;
using rmcs::util::AutoAimState;
using rmcs::util::Clock;
using rmcs::util::ControlState;

TEST(FeishuIntegration, BidirectionalCommunication) {
    const auto pid = ::fork();
    ASSERT_NE(pid, -1);

    if (pid == 0) {
        auto feishu_child = Feishu<RuntimeRole::AutoAim> {};
        std::this_thread::sleep_for(50ms); // ensure shm is ready

        auto deadline = Clock::now() + 500ms;
        auto ctrl     = std::optional<ControlState> {};
        while (!ctrl && Clock::now() < deadline) {
            if (feishu_child.updated()) {
                ctrl = feishu_child.fetch();
            } else {
                std::this_thread::sleep_for(10ms);
            }
        }
        ASSERT_TRUE(ctrl.has_value());

        auto auto_state            = AutoAimState {};
        auto_state.gimbal_takeover = true;
        auto_state.shoot_permitted = true;
        auto_state.yaw             = 1.23;

        ASSERT_TRUE(feishu_child.commit(auto_state));
        exit(0);
    }

    auto feishu_parent = Feishu<RuntimeRole::Control> {};
    std::this_thread::sleep_for(50ms); // ensure shm is ready

    auto ctrl         = ControlState {};
    ctrl.bullet_speed = 42.0;
    ctrl.timestamp    = Clock::now();

    ASSERT_TRUE(feishu_parent.commit(ctrl));

    auto deadline   = Clock::now() + 500ms;
    auto auto_state = std::optional<AutoAimState> {};
    while (!auto_state && Clock::now() < deadline) {
        if (feishu_parent.updated()) {
            auto_state = feishu_parent.fetch();
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
