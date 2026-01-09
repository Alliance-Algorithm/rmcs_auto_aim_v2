#include "utility/robot/id.hpp"
#include <gtest/gtest.h>

using namespace rmcs;

// -------------------- 编译期测试 --------------------
constexpr auto generate_devices() {
    auto devices = DeviceIds {
        DeviceId::INFANTRY_3,
        DeviceId::INFANTRY_4,
    };
    devices.append(DeviceId::SENTRY);
    devices.remove(DeviceId::INFANTRY_4);
    return devices;
}

TEST(device_id, compile_time) {
    constexpr auto devices = generate_devices();
    static_assert(devices.contains(DeviceId::INFANTRY_3));
    static_assert(devices.contains(DeviceId::SENTRY));
    static_assert(!devices.contains(DeviceId::INFANTRY_4));
    static_assert(!devices.empty());
}

// -------------------- 运行时测试 --------------------
TEST(device_id, runtime_operations) {
    DeviceIds d1 { DeviceId::HERO };
    EXPECT_TRUE(d1.contains(DeviceId::HERO));
    EXPECT_FALSE(d1.contains(DeviceId::ENGINEER));

    d1.append(DeviceId::ENGINEER);
    EXPECT_TRUE(d1.contains(DeviceId::ENGINEER));

    d1.remove(DeviceId::HERO);
    EXPECT_FALSE(d1.contains(DeviceId::HERO));
    EXPECT_TRUE(d1.contains(DeviceId::ENGINEER));
}

TEST(device_id, bitwise_operators) {
    DeviceIds d1 { DeviceId::HERO };
    DeviceIds d2 { DeviceId::ENGINEER };

    auto d_or = d1 | d2;
    EXPECT_TRUE(d_or.contains(DeviceId::HERO));
    EXPECT_TRUE(d_or.contains(DeviceId::ENGINEER));

    auto d_and = d1 & d2;
    EXPECT_FALSE(d_and.contains(DeviceId::HERO));
    EXPECT_FALSE(d_and.contains(DeviceId::ENGINEER));
    EXPECT_TRUE(d_and.empty());
}

TEST(device_id, predefined_groups) {
    auto large = DeviceIds::kLargeArmor();
    EXPECT_TRUE(large.contains(DeviceId::HERO));
    EXPECT_TRUE(large.contains(DeviceId::ENGINEER));
    EXPECT_TRUE(large.contains(DeviceId::BASE));
    EXPECT_FALSE(large.contains(DeviceId::INFANTRY_3));

    auto small = DeviceIds::kSmallArmor();
    EXPECT_TRUE(small.contains(DeviceId::INFANTRY_3));
    EXPECT_TRUE(small.contains(DeviceId::OUTPOST));
    EXPECT_FALSE(small.contains(DeviceId::BASE));
}

// -------------------- 参数化测试 --------------------
struct DeviceIdParam {
    DeviceId id;
    std::size_t index;
    std::string_view name;
};

class DeviceIdTest : public ::testing::TestWithParam<DeviceIdParam> { };

TEST_P(DeviceIdTest, to_index_and_to_string_consistency) {
    auto param = GetParam();
    EXPECT_EQ(to_index(param.id), param.index);
    EXPECT_EQ(to_string(param.id), param.name);
    EXPECT_EQ(from_index(param.index), param.id);
}

INSTANTIATE_TEST_SUITE_P(AllDeviceIds, DeviceIdTest,
    ::testing::Values( //
        DeviceIdParam { DeviceId::UNKNOWN, 0, "UNKNOWN" },
        DeviceIdParam { DeviceId::HERO, 1, "HERO" },
        DeviceIdParam { DeviceId::ENGINEER, 2, "ENGINEER" },
        DeviceIdParam { DeviceId::INFANTRY_3, 3, "INFANTRY_3" },
        DeviceIdParam { DeviceId::INFANTRY_4, 4, "INFANTRY_4" },
        DeviceIdParam { DeviceId::INFANTRY_5, 5, "INFANTRY_5" },
        DeviceIdParam { DeviceId::AERIAL, 6, "AERIAL" },
        DeviceIdParam { DeviceId::SENTRY, 7, "SENTRY" },
        DeviceIdParam { DeviceId::DART, 8, "DART" }, DeviceIdParam { DeviceId::RADAR, 9, "RADAR" },
        DeviceIdParam { DeviceId::OUTPOST, 10, "OUTPOST" },
        DeviceIdParam { DeviceId::BASE, 11, "BASE" }) //
);
