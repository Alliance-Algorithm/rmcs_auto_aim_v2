#include "utility/shared/interprocess.hpp"

#include <gtest/gtest.h>

#include <cstdlib>
#include <sys/wait.h>
#include <thread>

TEST(shm, shm) {
    using Send = rmcs::shm::Client<std::uint64_t>::Send;
    using Recv = rmcs::shm::Client<std::uint64_t>::Recv;

    using namespace std::chrono_literals;

    constexpr auto shm_name      = "/rmcs_auto_aim_test";
    constexpr auto test_value    = std::uint64_t { 42 };
    constexpr auto max_attempts  = 100;
    constexpr auto poll_interval = 10ms;
    constexpr auto init_delay    = 50ms;

    // 父进程先创建共享内存对象
    auto send = Send { };
    ASSERT_TRUE(send.open(shm_name));
    ASSERT_TRUE(send.opened());

    auto pid = fork();
    ASSERT_GE(pid, 0);

    if (pid == 0) {
        // 子进程：接收数据
        auto recv = Recv { };
        ASSERT_TRUE(recv.open(shm_name));
        ASSERT_TRUE(recv.opened());

        // 等待接收数据
        auto received_value = std::uint64_t { 0 };
        auto received       = false;

        // 尝试接收数据，最多等待一段时间
        for (auto i = 0; i < max_attempts; ++i) {
            if (recv.is_updated()) {
                recv.with_read([&](const auto& data) { received_value = data; });
                received = true;
                break;
            }
            std::this_thread::sleep_for(poll_interval);
        }

        ASSERT_TRUE(received);
        ASSERT_EQ(received_value, test_value);

        std::exit(0);
    } else {
        // 父进程：发送数据
        // 等待一下子进程准备好
        std::this_thread::sleep_for(init_delay);

        // 发送数据
        send.with_write([&](auto& data) { data = test_value; });

        // 等待子进程完成
        auto status = int { 0 };
        waitpid(pid, &status, 0);
        ASSERT_TRUE(WIFEXITED(status));
        ASSERT_EQ(WEXITSTATUS(status), 0);
    }
}
