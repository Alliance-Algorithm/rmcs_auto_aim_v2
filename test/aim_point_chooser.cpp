#include <gtest/gtest.h>

#include <eigen3/Eigen/Geometry>

#include <array>
#include <cmath>
#include <memory>
#include <optional>
#include <random>
#include <span>
#include <string>
#include <vector>

#include "module/fire_control/aim_point_chooser.hpp"
#include "utility/math/angle.hpp"

using rmcs::Armor3D;
using rmcs::ArmorColor;
using rmcs::DeviceId;
using rmcs::Orientation;
using rmcs::Translation;
using rmcs::fire_control::AimPointChooser;

namespace {

constexpr auto kAngleStepDeg   = 5;
constexpr auto kLowSpeed       = 1.0;
constexpr auto kFastPositive   = 4.0;
constexpr auto kFastNegative   = -4.0;
constexpr auto kSwitchSpeedEps = 0.01;

auto make_chooser() -> std::unique_ptr<AimPointChooser> {
    auto chooser = std::make_unique<AimPointChooser>();
    chooser->initialize(AimPointChooser::Config {
        .coming_angle               = rmcs::util::deg2rad(60.0),
        .leaving_angle              = rmcs::util::deg2rad(20.0),
        .angular_velocity_threshold = 2.0,
        .outpost_coming_angle       = rmcs::util::deg2rad(70.0),
        .outpost_leaving_angle      = rmcs::util::deg2rad(30.0),
    });
    return chooser;
}

auto make_center_position(double yaw_deg) -> Eigen::Vector3d {
    auto const yaw_rad = rmcs::util::deg2rad(yaw_deg);
    return {
        std::cos(yaw_rad),
        std::sin(yaw_rad),
        0.0,
    };
}

auto make_armor(double yaw_deg, int id, DeviceId genre) -> Armor3D {
    auto armor  = Armor3D {};
    armor.genre = genre;
    armor.color = ArmorColor::BLUE;
    armor.id    = id;

    auto const yaw_rad = rmcs::util::deg2rad(yaw_deg);
    auto const q       = Eigen::Quaterniond { Eigen::AngleAxisd { yaw_rad, Eigen::Vector3d::UnitZ() } };

    armor.translation = Translation { 0.0, 0.0, 0.0 };
    armor.orientation = Orientation { q };
    return armor;
}

auto make_armors(std::initializer_list<double> yaws_deg,
    DeviceId genre = DeviceId::SENTRY) -> std::vector<Armor3D> {
    auto armors = std::vector<Armor3D> {};
    armors.reserve(yaws_deg.size());

    auto index = 0;
    for (auto const yaw_deg : yaws_deg) {
        armors.emplace_back(make_armor(yaw_deg, index, genre));
        ++index;
    }
    return armors;
}

auto make_armors(std::span<double const> yaws_deg,
    DeviceId genre = DeviceId::SENTRY) -> std::vector<Armor3D> {
    auto armors = std::vector<Armor3D> {};
    armors.reserve(yaws_deg.size());

    for (size_t i = 0; i < yaws_deg.size(); ++i) {
        armors.emplace_back(make_armor(yaws_deg[i], static_cast<int>(i), genre));
    }
    return armors;
}

auto choose_id(AimPointChooser& chooser, std::span<Armor3D const> armors, double center_yaw_deg,
    double angular_velocity) -> std::optional<int> {
    auto const center = make_center_position(center_yaw_deg);
    auto const chosen = chooser.choose_armor(armors, center, angular_velocity);
    if (!chosen.has_value()) return std::nullopt;
    return chosen->id;
}

auto choose_once(std::span<Armor3D const> armors, double center_yaw_deg, double angular_velocity)
    -> std::optional<int> {
    auto chooser = make_chooser();
    return choose_id(*chooser, armors, center_yaw_deg, angular_velocity);
}

auto expect_single_armor_result(double angle_deg, double angular_velocity, DeviceId genre,
    std::optional<int> expected) -> void {
    auto const armors = make_armors({ angle_deg }, genre);
    auto const actual = choose_once(armors, 0.0, angular_velocity);
    EXPECT_EQ(actual, expected);
}

} // namespace

TEST(AimPointChooser, ScenarioTemplateSingleArmorLowSpeedAcquireScan) {
    for (int angle_deg = -180; angle_deg <= 180; angle_deg += kAngleStepDeg) {
        auto const armors = make_armors({ static_cast<double>(angle_deg) });
        auto const actual = choose_once(armors, 0.0, kLowSpeed);

        auto const expected = (std::abs(angle_deg) < 60) ? std::optional<int> { 0 } : std::nullopt;

        SCOPED_TRACE("angle_deg=" + std::to_string(angle_deg));
        EXPECT_EQ(actual, expected);
    }
}

TEST(AimPointChooser, ScenarioTemplateSingleArmorHighSpeedScanBySpinDirection) {
    constexpr std::array speeds { 2.0, kFastPositive, -2.0, kFastNegative };

    for (int angle_deg = -180; angle_deg <= 180; angle_deg += kAngleStepDeg) {
        auto const armors = make_armors({ static_cast<double>(angle_deg) });

        for (auto const speed : speeds) {
            auto const in_coming_window = std::abs(angle_deg) < 60;
            auto const in_leaving_window = (speed > 0.0) ? (angle_deg <= 20) : (angle_deg >= -20);
            auto const expected = (in_coming_window && in_leaving_window)
                ? std::optional<int> { 0 }
                : std::nullopt;

            auto const actual = choose_once(armors, 0.0, speed);

            SCOPED_TRACE("angle_deg=" + std::to_string(angle_deg));
            SCOPED_TRACE("speed=" + std::to_string(speed));
            EXPECT_EQ(actual, expected);
        }
    }
}

TEST(AimPointChooser, ScenarioTemplateSingleArmorOutpostScan) {
    constexpr std::array speeds { 0.0, 1.0, -1.0, 1.99, -1.99 };

    for (int angle_deg = -180; angle_deg <= 180; angle_deg += kAngleStepDeg) {
        for (auto const speed : speeds) {
            auto const in_coming_window = std::abs(angle_deg) < 70;

            auto in_leaving_window = false;
            if (speed == 0.0) {
                in_leaving_window = true;
            } else if (speed > 0.0) {
                in_leaving_window = angle_deg <= 30;
            } else {
                in_leaving_window = angle_deg >= -30;
            }

            auto const expected = (in_coming_window && in_leaving_window)
                ? std::optional<int> { 0 }
                : std::nullopt;

            auto const armors = make_armors({ static_cast<double>(angle_deg) }, DeviceId::OUTPOST);
            auto const actual = choose_once(armors, 0.0, speed);

            SCOPED_TRACE("angle_deg=" + std::to_string(angle_deg));
            SCOPED_TRACE("speed=" + std::to_string(speed));
            EXPECT_EQ(actual, expected);
        }
    }
}

TEST(AimPointChooser, BoundaryCasesCoverWindowsAndVelocityThreshold) {
    struct TestCase {
        std::string name;
        double angle_deg;
        double speed;
        DeviceId genre;
        std::optional<int> expected;
    };

    constexpr auto kDegEps = 0.001;

    auto const cases = std::array {
        TestCase { "low_speed_acquire_inside", 60.0 - kDegEps, kLowSpeed, DeviceId::SENTRY, 0 },
        TestCase { "low_speed_acquire_boundary", 60.0, kLowSpeed, DeviceId::SENTRY, std::nullopt },
        TestCase { "low_speed_acquire_outside", 60.0 + kDegEps, kLowSpeed, DeviceId::SENTRY,
            std::nullopt },

        TestCase { "high_positive_leaving_boundary", 20.0, kFastPositive, DeviceId::SENTRY, 0 },
        TestCase { "high_positive_leaving_outside", 20.0 + kDegEps, kFastPositive,
            DeviceId::SENTRY, std::nullopt },
        TestCase { "high_positive_coming_inside", -60.0 + kDegEps, kFastPositive,
            DeviceId::SENTRY, 0 },
        TestCase { "high_positive_coming_boundary", -60.0, kFastPositive, DeviceId::SENTRY,
            std::nullopt },

        TestCase { "high_negative_leaving_boundary", -20.0, kFastNegative, DeviceId::SENTRY, 0 },
        TestCase { "high_negative_leaving_outside", -20.0 - kDegEps, kFastNegative,
            DeviceId::SENTRY, std::nullopt },
        TestCase { "high_negative_coming_inside", 60.0 - kDegEps, kFastNegative, DeviceId::SENTRY,
            0 },
        TestCase { "high_negative_coming_boundary", 60.0, kFastNegative, DeviceId::SENTRY,
            std::nullopt },

        TestCase { "outpost_positive_leaving_boundary", 30.0, 1.0, DeviceId::OUTPOST, 0 },
        TestCase { "outpost_positive_leaving_outside", 30.0 + kDegEps, 1.0, DeviceId::OUTPOST,
            std::nullopt },
        TestCase { "outpost_positive_coming_inside", -70.0 + kDegEps, 1.0, DeviceId::OUTPOST, 0 },
        TestCase { "outpost_positive_coming_boundary", -70.0, 1.0, DeviceId::OUTPOST,
            std::nullopt },

        TestCase { "velocity_threshold_below", 50.0, 2.0 - kSwitchSpeedEps, DeviceId::SENTRY, 0 },
        TestCase { "velocity_threshold_equal", 50.0, 2.0, DeviceId::SENTRY, std::nullopt },
        TestCase { "velocity_threshold_above", 50.0, 2.0 + kSwitchSpeedEps, DeviceId::SENTRY,
            std::nullopt },
        TestCase {
            "velocity_threshold_negative_below", -50.0, -2.0 + kSwitchSpeedEps, DeviceId::SENTRY, 0 },
        TestCase { "velocity_threshold_negative_equal", -50.0, -2.0, DeviceId::SENTRY,
            std::nullopt },
        TestCase { "velocity_threshold_negative_above", -50.0, -2.0 - kSwitchSpeedEps,
            DeviceId::SENTRY, std::nullopt },
    };

    for (auto const& test_case : cases) {
        SCOPED_TRACE(test_case.name);
        expect_single_armor_result(
            test_case.angle_deg, test_case.speed, test_case.genre, test_case.expected);
    }
}

TEST(AimPointChooser, SymmetricDualArmorHighSpeedFollowsIncomingDirection) {
    for (int theta_deg = 5; theta_deg <= 55; theta_deg += 5) {
        auto const armors = make_armors({ -static_cast<double>(theta_deg), static_cast<double>(theta_deg) });

        auto const positive_spin = choose_once(armors, 0.0, kFastPositive);
        auto const negative_spin = choose_once(armors, 0.0, kFastNegative);

        SCOPED_TRACE("theta_deg=" + std::to_string(theta_deg));
        EXPECT_EQ(positive_spin, std::optional<int> { 0 });
        EXPECT_EQ(negative_spin, std::optional<int> { 1 });
    }
}

TEST(AimPointChooser, SymmetricLowSpeedSwitchUsesMarginAndImmediateSwitch) {
    auto chooser = make_chooser();

    auto const frame_initial = make_armors({ -30.0, 30.0 });
    EXPECT_EQ(choose_id(*chooser, frame_initial, 0.0, kLowSpeed), std::optional<int> { 0 });

    auto const frame_margin = make_armors({ -30.0, 25.0 }); // improvement = 5 deg
    EXPECT_EQ(choose_id(*chooser, frame_margin, 0.0, kLowSpeed), std::optional<int> { 0 });

    auto const frame_switch = make_armors({ -30.0, 22.0 }); // improvement = 8 deg
    EXPECT_EQ(choose_id(*chooser, frame_switch, 0.0, kLowSpeed), std::optional<int> { 1 });
}

TEST(AimPointChooser, LowSpeedSwitchesImmediatelyWhenPreferredTargetChanges) {
    auto chooser = make_chooser();

    auto const frame_initial = make_armors({ 10.0, 30.0, 40.0 });
    EXPECT_EQ(choose_id(*chooser, frame_initial, 0.0, kLowSpeed), std::optional<int> { 0 });

    auto const frame_target_2 = make_armors({ 20.0, 15.0, 8.0 });
    EXPECT_EQ(choose_id(*chooser, frame_target_2, 0.0, kLowSpeed), std::optional<int> { 2 });

    auto const frame_target_1 = make_armors({ 20.0, 8.0, 15.0 });
    EXPECT_EQ(choose_id(*chooser, frame_target_1, 0.0, kLowSpeed), std::optional<int> { 1 });
}

TEST(AimPointChooser, HighSpeedSwitchesImmediatelyWhenImprovementExceedsMargin) {
    auto chooser = make_chooser();

    auto const frame_initial = make_armors({ 12.0, 18.0, 25.0 });
    EXPECT_EQ(choose_id(*chooser, frame_initial, 0.0, kFastPositive), std::optional<int> { 0 });

    auto const frame_switch = make_armors({ 20.0, 12.0, 25.0 }); // abs improvement = 8 deg
    EXPECT_EQ(choose_id(*chooser, frame_switch, 0.0, kFastPositive), std::optional<int> { 1 });
}

TEST(AimPointChooser, EmptyInputClearsStateAndAllowsImmediateReacquire) {
    auto chooser = make_chooser();

    auto const frame_a = make_armors({ 10.0, 20.0 });
    auto const frame_b = make_armors({ 20.0, 10.0 });
    auto const frame_no_acquire = make_armors({ 70.0, 65.0 });
    auto const empty   = std::vector<Armor3D> {};

    EXPECT_EQ(choose_id(*chooser, frame_a, 0.0, kLowSpeed), std::optional<int> { 0 });
    EXPECT_EQ(choose_id(*chooser, frame_b, 0.0, kLowSpeed), std::optional<int> { 1 });
    EXPECT_EQ(choose_id(*chooser, frame_no_acquire, 0.0, kLowSpeed), std::nullopt);

    EXPECT_EQ(choose_id(*chooser, empty, 0.0, kLowSpeed), std::nullopt);
    EXPECT_EQ(choose_id(*chooser, frame_no_acquire, 0.0, kLowSpeed), std::nullopt);
}

TEST(AimPointChooser, YawWraparoundAndGlobalShiftAreInvariant) {
    constexpr std::array speeds { kLowSpeed, kFastPositive, kFastNegative };

    for (int angle_deg = -175; angle_deg <= 175; angle_deg += 35) {
        for (auto const speed : speeds) {
            auto const base_armors  = make_armors({ static_cast<double>(angle_deg) });
            auto const plus_armors  = make_armors({ static_cast<double>(angle_deg + 360) });
            auto const minus_armors = make_armors({ static_cast<double>(angle_deg - 360) });

            auto const base_result  = choose_once(base_armors, 0.0, speed);
            auto const plus_result  = choose_once(plus_armors, 0.0, speed);
            auto const minus_result = choose_once(minus_armors, 0.0, speed);

            SCOPED_TRACE("angle_deg=" + std::to_string(angle_deg));
            SCOPED_TRACE("speed=" + std::to_string(speed));

            EXPECT_EQ(base_result, plus_result);
            EXPECT_EQ(base_result, minus_result);
        }
    }

    constexpr std::array<double, 3> template_yaws { -35.0, 15.0, 80.0 };
    constexpr std::array genres { DeviceId::SENTRY, DeviceId::OUTPOST };

    for (auto const genre : genres) {
        for (auto const speed : speeds) {
            auto const baseline_armors  = make_armors(template_yaws, genre);
            auto const baseline_result  = choose_once(baseline_armors, 0.0, speed);

            for (int shift_deg = -150; shift_deg <= 150; shift_deg += 30) {
                auto shifted_yaws = std::vector<double> {};
                shifted_yaws.reserve(template_yaws.size());
                for (auto const yaw_deg : template_yaws) {
                    shifted_yaws.push_back(yaw_deg + static_cast<double>(shift_deg));
                }

                auto const shifted_armors = make_armors(shifted_yaws, genre);
                auto const shifted_result =
                    choose_once(shifted_armors, static_cast<double>(shift_deg), speed);

                SCOPED_TRACE("genre=" + std::to_string(static_cast<int>(genre)));
                SCOPED_TRACE("speed=" + std::to_string(speed));
                SCOPED_TRACE("shift_deg=" + std::to_string(shift_deg));

                EXPECT_EQ(shifted_result, baseline_result);
            }
        }
    }
}

TEST(AimPointChooser, RandomizedScenarioRegressionIsDeterministicAndValid) {
    struct Frame {
        std::vector<Armor3D> armors;
        double center_yaw_deg;
        double angular_velocity;
    };

    auto rng = std::mt19937 { 0xA11C0DEu };

    auto count_dist       = std::uniform_int_distribution<int> { 0, 4 };
    auto yaw_dist         = std::uniform_real_distribution<double> { -720.0, 720.0 };
    auto center_yaw_dist  = std::uniform_real_distribution<double> { -180.0, 180.0 };
    auto velocity_dist    = std::uniform_real_distribution<double> { -8.0, 8.0 };
    auto outpost_selector = std::bernoulli_distribution { 0.25 };

    auto frames = std::vector<Frame> {};
    frames.reserve(2000);

    for (int i = 0; i < 2000; ++i) {
        auto const count = count_dist(rng);
        auto const genre = outpost_selector(rng) ? DeviceId::OUTPOST : DeviceId::SENTRY;

        auto armors = std::vector<Armor3D> {};
        armors.reserve(static_cast<size_t>(count));

        for (int id = 0; id < count; ++id) {
            armors.emplace_back(make_armor(yaw_dist(rng), id, genre));
        }

        frames.emplace_back(Frame {
            .armors           = std::move(armors),
            .center_yaw_deg   = center_yaw_dist(rng),
            .angular_velocity = velocity_dist(rng),
        });
    }

    auto chooser_a = make_chooser();
    auto chooser_b = make_chooser();

    for (size_t i = 0; i < frames.size(); ++i) {
        auto const& frame = frames[i];

        auto const result_a = choose_id(
            *chooser_a, frame.armors, frame.center_yaw_deg, frame.angular_velocity);
        auto const result_b = choose_id(
            *chooser_b, frame.armors, frame.center_yaw_deg, frame.angular_velocity);

        SCOPED_TRACE("frame_index=" + std::to_string(i));
        EXPECT_EQ(result_a, result_b);

        if (frame.armors.empty()) {
            EXPECT_EQ(result_a, std::nullopt);
        }

        if (result_a.has_value()) {
            EXPECT_GE(*result_a, 0);
            EXPECT_LT(*result_a, static_cast<int>(frame.armors.size()));
        }
    }
}
