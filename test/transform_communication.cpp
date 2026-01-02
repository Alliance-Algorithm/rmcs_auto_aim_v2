#include <eigen3/Eigen/Dense>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

#include "utility/shared/context.hpp"
#include "utility/shared/interprocess.hpp"
#include <gtest/gtest.h>

using Transform = rmcs::util::Transform;

namespace {

constexpr auto create_input_transform() -> Transform {
    using rmcs::Direction3d;
    using rmcs::Orientation;
    return {
        .position    = Direction3d { 1.0, 2.0, 3.0 },
        .orientation = Orientation { 0.0, 0.0, 0.0, 1.0 },
    };
}

// 非期望值，用于验证接收后被正确覆盖
inline auto create_failed_input_transform() -> Transform {
    using rmcs::Direction3d;
    using rmcs::Orientation;
    return {
        .position    = Direction3d { 2.0, 3.0, 3.0 },
        .orientation = Orientation { Eigen::Quaterniond::Identity() },
    };
}

inline auto expect_transform_equal(const Transform& lhs, const Transform& rhs) -> void {
    EXPECT_DOUBLE_EQ(lhs.position.x, rhs.position.x);
    EXPECT_DOUBLE_EQ(lhs.position.y, rhs.position.y);
    EXPECT_DOUBLE_EQ(lhs.position.z, rhs.position.z);
    EXPECT_DOUBLE_EQ(lhs.orientation.x, rhs.orientation.x);
    EXPECT_DOUBLE_EQ(lhs.orientation.y, rhs.orientation.y);
    EXPECT_DOUBLE_EQ(lhs.orientation.z, rhs.orientation.z);
    EXPECT_DOUBLE_EQ(lhs.orientation.w, rhs.orientation.w);
}

} // namespace

// 父子进程通信：确保写入的 Transform 能被完整读取
TEST(TransformShm, SendRecvSequence) {
    using Send = rmcs::shm::Client<Transform>::Send;
    using Recv = rmcs::shm::Client<Transform>::Recv;

    using namespace std::chrono_literals;

    constexpr auto shm_name      = "/rmcs_auto_aim_transform_shm_test";
    constexpr auto test_value    = create_input_transform();
    constexpr auto max_attempts  = 100;
    constexpr auto poll_interval = 10ms;
    constexpr auto init_delay    = 50ms;

    // 父进程先创建共享内存对象
    auto send = Send {};
    ASSERT_TRUE(send.open(shm_name));
    ASSERT_TRUE(send.opened());

    auto pid = fork();
    ASSERT_GE(pid, 0);

    if (pid == 0) {
        // 子进程：接收数据
        auto recv = Recv {};
        ASSERT_TRUE(recv.open(shm_name));
        ASSERT_TRUE(recv.opened());

        // 等待接收数据
        auto received_value = create_failed_input_transform();
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

        ASSERT_TRUE(received && "failed to receive");

        expect_transform_equal(received_value, test_value);

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

// 单进程读写：验证版本控制与快照读取的正确性，模拟 component 中动态变换的发布/消费
TEST(TransformShm, SnapshotAndUpdate) {
    using Client = rmcs::shm::Client<Transform>;
    Client::Send send;
    Client::Recv recv;

    constexpr auto shm_name = "/rmcs_auto_aim_snapshot_test";

    ASSERT_TRUE(send.open(shm_name));
    ASSERT_TRUE(recv.open(shm_name));

    const auto first  = create_input_transform();
    const auto second = create_failed_input_transform();

    // 初次写入，接收端应检测到更新
    send.send(first);
    EXPECT_TRUE(recv.is_updated());

    Transform snapshot {};
    recv.with_read([&](const auto& data) { snapshot = data; });
    expect_transform_equal(snapshot, first);
    EXPECT_FALSE(recv.is_updated()); // 读取后版本同步

    // 再写入一次不同数据，接收端应再次检测到更新
    send.with_write([&](auto& data) { data = second; });
    EXPECT_TRUE(recv.is_updated());

    recv.recv(snapshot); // 使用 recv 接口读取一次
    expect_transform_equal(snapshot, second);
    EXPECT_FALSE(recv.is_updated());
}
