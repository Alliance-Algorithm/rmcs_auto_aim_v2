#include <array>
#include <cmath>
#include <initializer_list>
#include <memory>
#include <optional>
#include <random>
#include <span>
#include <string>
#include <vector>

#include <eigen3/Eigen/Geometry>
#include <gtest/gtest.h>

#include "module/fire_control/aim_point_chooser.hpp"
#include "utility/math/angle.hpp"

using rmcs::Armor3D;
using rmcs::ArmorColor;
using rmcs::DeviceId;
using rmcs::Orientation;
using rmcs::Translation;
using rmcs::fire_control::AimPointChooser;

namespace {

constexpr auto kAngleStepDeg = 5;
constexpr auto kFastPositive = 4.0;
constexpr auto kFastNegative = -4.0;

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
    return { std::cos(yaw_rad), std::sin(yaw_rad), 0.0 };
}

auto make_armor(double yaw_deg, int id, DeviceId genre) -> Armor3D {
    auto armor  = Armor3D {};
    armor.genre = genre;
    armor.color = ArmorColor::BLUE;
    armor.id    = id;

    auto const yaw_rad = rmcs::util::deg2rad(yaw_deg);
    auto const q = Eigen::Quaterniond { Eigen::AngleAxisd { yaw_rad, Eigen::Vector3d::UnitZ() } };

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

} // namespace

TEST(AimPointChooser, SingleArmorHighSpeedScanBySpinDirection) {
    for (int angle_deg = -180; angle_deg <= 180; angle_deg += kAngleStepDeg) {
        auto const armors = make_armors({ static_cast<double>(angle_deg) });

        for (auto const speed : std::array { 2.0, kFastPositive, -2.0, kFastNegative }) {
            auto const in_coming_window  = std::abs(angle_deg) <= 60;
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

TEST(AimPointChooser, PreferIncomingArmorWhenSpinPositive) {
    auto const armors = make_armors({ -30.0, 5.0 });
    auto const chosen = choose_once(armors, 0.0, kFastPositive);
    EXPECT_EQ(chosen, std::optional<int> { 0 });
}

TEST(AimPointChooser, PreferIncomingArmorWhenSpinNegative) {
    auto const armors = make_armors({ -5.0, 30.0 });
    auto const chosen = choose_once(armors, 0.0, kFastNegative);
    EXPECT_EQ(chosen, std::optional<int> { 1 });
}

TEST(AimPointChooser, KeepAbsoluteDeltaPriorityWhenSpinZero) {
    auto const armors = make_armors({ -15.0, 5.0 });
    auto const chosen = choose_once(armors, 0.0, 0.0);
    EXPECT_EQ(chosen, std::optional<int> { 1 });
}
