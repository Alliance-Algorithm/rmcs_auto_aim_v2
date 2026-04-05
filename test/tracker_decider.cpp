#include "module/predictor/robot_state.hpp"
#include "module/tracker/decider.hpp"
#include "utility/math/conversion.hpp"

#include <chrono>
#include <numbers>
#include <vector>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

using namespace rmcs;
using namespace rmcs::tracker;
using namespace std::chrono_literals;

namespace {
auto make_decider_config() -> YAML::Node {
    return YAML::Load(R"(
max_temporary_loss_frames: 5
max_detecting_loss_frames: 3
tracking_confirm_frames: 3
)");
}

auto make_armor(double x, double y, double z, double yaw,
    DeviceId id = DeviceId::INFANTRY_3) -> Armor3D {
    auto armor        = Armor3D {};
    armor.genre       = id;
    armor.color       = ArmorColor::BLUE;
    armor.id          = 0;
    armor.translation = Translation { x, y, z };
    armor.orientation = util::euler_to_quaternion(yaw, 0., 0.);
    return armor;
}
}

TEST(robot_state, update_rejects_mismatched_measurement) {
    auto state = predictor::RobotState {};
    auto t     = predictor::RobotState::Clock::now();

    auto tracked = make_armor(5.0, 0.0, 0.0, 0.0);
    state.initialize(tracked, t);

    t += 8ms;
    state.predict(t);

    auto mismatched = make_armor(-4.0, 4.0, 0.0, 0.0);
    EXPECT_FALSE(state.update(mismatched));
}

TEST(tracker_decider, reacquire_requires_confirm_frames) {
    auto decider = Decider {};
    auto init    = decider.initialize(make_decider_config());
    ASSERT_TRUE(init.has_value()) << init.error();

    auto t     = Decider::Clock::now();
    auto armor = make_armor(5.0, 0.0, 0.0, 0.0);

    auto observed = std::vector<Armor3D> { armor };
    auto empty    = std::vector<Armor3D> {};

    Decider::Output output {};
    for (int i = 0; i < 12; ++i) {
        t += 8ms;
        output = decider.update(observed, t);
    }
    EXPECT_TRUE(output.allow_takeover);
    EXPECT_TRUE(output.tracking_confirmed);

    for (int i = 0; i < 2; ++i) {
        t += 8ms;
        output = decider.update(empty, t);
    }
    EXPECT_TRUE(output.allow_takeover);
    EXPECT_FALSE(output.tracking_confirmed);

    t += 8ms;
    output = decider.update(observed, t);
    EXPECT_FALSE(output.allow_takeover);
    EXPECT_FALSE(output.tracking_confirmed);

    t += 8ms;
    output = decider.update(observed, t);
    EXPECT_FALSE(output.allow_takeover);
    EXPECT_FALSE(output.tracking_confirmed);

    t += 8ms;
    output = decider.update(observed, t);
    EXPECT_TRUE(output.allow_takeover);
    EXPECT_TRUE(output.tracking_confirmed);
}

TEST(tracker_decider, invalid_observation_is_not_treated_as_seen) {
    auto decider = Decider {};
    auto init    = decider.initialize(make_decider_config());
    ASSERT_TRUE(init.has_value()) << init.error();

    auto t      = Decider::Clock::now();
    auto valid  = make_armor(5.0, 0.0, 0.0, 0.0);
    auto bogus  = make_armor(-4.0, 4.0, 0.0, std::numbers::pi);
    auto seen   = std::vector<Armor3D> { valid };
    auto unseen = std::vector<Armor3D> { bogus };

    Decider::Output output {};
    for (int i = 0; i < 12; ++i) {
        t += 8ms;
        output = decider.update(seen, t);
    }
    ASSERT_TRUE(output.allow_takeover);
    ASSERT_TRUE(output.tracking_confirmed);

    t += 8ms;
    output = decider.update(unseen, t);
    EXPECT_TRUE(output.allow_takeover);
    EXPECT_FALSE(output.tracking_confirmed);

    for (int i = 0; i < 5; ++i) {
        t += 8ms;
        output = decider.update(unseen, t);
    }
    EXPECT_EQ(output.target_id, DeviceId::UNKNOWN);
    EXPECT_FALSE(output.snapshot.has_value());
    EXPECT_FALSE(output.allow_takeover);
    EXPECT_FALSE(output.tracking_confirmed);
}
