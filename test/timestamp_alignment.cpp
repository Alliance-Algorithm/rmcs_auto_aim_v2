#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <sys/mman.h>
#include <type_traits>
#include <unistd.h>

#include "kernel/feishu.hpp"
#include "utility/shared/interprocess.hpp"

namespace {

using namespace std::chrono_literals;

using rmcs::kernel::Feishu;
using rmcs::kernel::RuntimeRole;
using rmcs::shm::HistoryClient;
using rmcs::util::CameraTriggerEvent;
using rmcs::util::Clock;
using rmcs::util::ControlState;

struct ShmScope {
    explicit ShmScope(std::string name)
        : name_(std::move(name)) {
        (void)::shm_unlink(name_.c_str());
    }

    ~ShmScope() { (void)::shm_unlink(name_.c_str()); }

    [[nodiscard]] auto c_str() const noexcept -> const char* { return name_.c_str(); }

private:
    std::string name_;
};

struct FeishuShmScope {
    FeishuShmScope() {
        for (auto name : kNames) {
            (void)::shm_unlink(name);
        }
    }

    ~FeishuShmScope() {
        for (auto name : kNames) {
            (void)::shm_unlink(name);
        }
    }

private:
    static constexpr std::array kNames {
        rmcs::kernel::shm_name<rmcs::util::AutoAimState>,
        rmcs::kernel::shm_name<rmcs::util::ControlState>,
        rmcs::kernel::shm_name<rmcs::util::CameraTriggerEvent>,
    };
};

auto unique_shm_name(const char* prefix) -> std::string {
    static auto counter = std::atomic<std::uint64_t> { 0 };
    return std::string { "/" } + prefix + "_" + std::to_string(::getpid()) + "_"
        + std::to_string(counter.fetch_add(1, std::memory_order_relaxed));
}

auto make_control_state(Clock::time_point timestamp) -> ControlState {
    auto state      = ControlState {};
    state.timestamp = timestamp;
    return state;
}

TEST(TimestampAlignment, ClockDomainUsesSteadyClock) {
    EXPECT_TRUE((std::is_same_v<Clock, std::chrono::steady_clock>));
}

TEST(TimestampAlignment, HistoryClientPopNextConsumesCameraTriggersInOrder) {
    using Channel = HistoryClient<CameraTriggerEvent, 8>;

    auto shm_name = ShmScope { unique_shm_name("rmcs_auto_aim_trigger_fifo") };
    auto send     = Channel::Send {};
    auto recv     = Channel::Recv {};

    ASSERT_TRUE(send.open(shm_name.c_str()));
    ASSERT_TRUE(recv.open(shm_name.c_str()));

    auto base = Clock::now();
    ASSERT_TRUE(send.push(CameraTriggerEvent { .seq = 11, .timestamp = base + 1ms }));
    ASSERT_TRUE(send.push(CameraTriggerEvent { .seq = 12, .timestamp = base + 2ms }));
    ASSERT_TRUE(send.push(CameraTriggerEvent { .seq = 13, .timestamp = base + 3ms }));

    auto event = CameraTriggerEvent {};
    ASSERT_TRUE(recv.pop_next(event));
    EXPECT_EQ(event.seq, 11U);
    EXPECT_EQ(event.timestamp, base + 1ms);

    ASSERT_TRUE(recv.pop_next(event));
    EXPECT_EQ(event.seq, 12U);
    EXPECT_EQ(event.timestamp, base + 2ms);

    ASSERT_TRUE(recv.pop_next(event));
    EXPECT_EQ(event.seq, 13U);
    EXPECT_EQ(event.timestamp, base + 3ms);

    EXPECT_FALSE(recv.pop_next(event));
}

TEST(TimestampAlignment, HistoryClientFindLatestChoosesNewestStateNotAfterImageTimestamp) {
    using Channel = HistoryClient<ControlState, 8>;

    auto shm_name = ShmScope { unique_shm_name("rmcs_auto_aim_control_history") };
    auto send     = Channel::Send {};
    auto recv     = Channel::Recv {};

    ASSERT_TRUE(send.open(shm_name.c_str()));
    ASSERT_TRUE(recv.open(shm_name.c_str()));

    auto base = Clock::now();
    ASSERT_TRUE(send.push(make_control_state(base + 10ms)));
    ASSERT_TRUE(send.push(make_control_state(base + 20ms)));
    ASSERT_TRUE(send.push(make_control_state(base + 30ms)));

    auto matched = ControlState {};
    ASSERT_TRUE(recv.find_latest(
        [&](const ControlState& state) { return state.timestamp <= base + 25ms; }, matched));
    EXPECT_EQ(matched.timestamp, base + 20ms);

    ASSERT_TRUE(recv.find_latest(
        [&](const ControlState& state) { return state.timestamp <= base + 10ms; }, matched));
    EXPECT_EQ(matched.timestamp, base + 10ms);

    EXPECT_FALSE(
        recv.find_latest([&](const ControlState& state) { return state.timestamp < base; }, matched));
}

TEST(TimestampAlignment, LateTriggerEventMustNotBindToNextFrame) {
    using Channel = HistoryClient<CameraTriggerEvent, 8>;

    auto shm_name = ShmScope { unique_shm_name("rmcs_auto_aim_trigger_late") };
    auto send     = Channel::Send {};
    auto recv     = Channel::Recv {};

    ASSERT_TRUE(send.open(shm_name.c_str()));
    ASSERT_TRUE(recv.open(shm_name.c_str()));

    auto last_bound_trigger_seq        = std::uint64_t { 0 };
    auto last_image_capture_timestamp  = std::optional<Clock::time_point> {};
    constexpr auto trigger_sync_max_age = 50ms;

    auto bind = [&](Clock::time_point capture_timestamp) -> std::optional<CameraTriggerEvent> {
        auto trigger = CameraTriggerEvent {};
        auto ok      = recv.find_latest(
            [&](const CameraTriggerEvent& candidate) {
                return candidate.seq > last_bound_trigger_seq
                    && (!last_image_capture_timestamp
                        || candidate.timestamp > *last_image_capture_timestamp)
                    && candidate.timestamp <= capture_timestamp
                    && capture_timestamp - candidate.timestamp <= trigger_sync_max_age;
            },
            trigger);

        if (ok) {
            last_bound_trigger_seq       = trigger.seq;
            last_image_capture_timestamp = capture_timestamp;
            return trigger;
        }

        last_image_capture_timestamp = capture_timestamp;
        return std::nullopt;
    };

    auto base = Clock::now();

    EXPECT_FALSE(bind(base + 15ms).has_value());

    ASSERT_TRUE(send.push(CameraTriggerEvent { .seq = 100, .timestamp = base + 10ms }));
    EXPECT_FALSE(bind(base + 30ms).has_value());

    ASSERT_TRUE(send.push(CameraTriggerEvent { .seq = 101, .timestamp = base + 38ms }));
    auto matched = bind(base + 40ms);
    ASSERT_TRUE(matched.has_value());
    EXPECT_EQ(matched->seq, 101U);
    EXPECT_EQ(matched->timestamp, base + 38ms);
}

TEST(TimestampAlignment, FeishuFetchLatestBeforeUsesControlStateHistorySemantics) {
    auto shm_scope = FeishuShmScope {};
    auto control   = Feishu<RuntimeRole::Control> {};
    auto auto_aim  = Feishu<RuntimeRole::AutoAim> {};

    auto base = Clock::now();
    ASSERT_TRUE(control.commit(make_control_state(base + 10ms)));
    ASSERT_TRUE(control.commit(make_control_state(base + 20ms)));
    ASSERT_TRUE(control.commit(make_control_state(base + 30ms)));

    auto matched = auto_aim.fetch_latest_before(base + 25ms);
    ASSERT_TRUE(matched.has_value());
    EXPECT_EQ(matched->timestamp, base + 20ms);

    matched = auto_aim.fetch_latest_before(base + 30ms);
    ASSERT_TRUE(matched.has_value());
    EXPECT_EQ(matched->timestamp, base + 30ms);

    EXPECT_FALSE(auto_aim.fetch_latest_before(base + 5ms).has_value());
}

} // namespace
