#include "utility/math/angle.hpp"
#include "utility/math/conversion.hpp"
#include "utility/math/outpost.hpp"
#include "utility/rclcpp/node.hpp"
#include "utility/rclcpp/visual/armor.hpp"
#include "utility/robot/constant.hpp"

#include <array>
#include <chrono>
#include <memory>
#include <numbers>
#include <print>

#include <eigen3/Eigen/Geometry>

#include <rclcpp/utilities.hpp>

using namespace rmcs;
using namespace rmcs::util;
using namespace rmcs::util::visual;

auto main() -> int {
    using namespace std::chrono_literals;

    const auto source_yaw = deg2rad(25.0);

    auto visual = RclcppNode { "see_outpost" };
    visual.set_pub_topic_prefix("/rmcs/auto_aim/");

    auto source = Armor3D {
        .genre = DeviceId::OUTPOST,
        .color = ArmorColor::BLUE,
        .id    = 0,
        .translation =
            Translation {
                1.0 - kOutpostRadius * std::cos(source_yaw),
                0.5 - kOutpostRadius * std::sin(source_yaw),
                1.2,
            },
        .orientation =
            Orientation { euler_to_quaternion(source_yaw, kPredictedOutpostArmorPitch, 0.0) },
    };

    auto right_solution           = NeighborBarSolution { };
    right_solution.input.source   = source;
    right_solution.input.in_right = true;
    right_solution.solve();

    auto left_solution           = NeighborBarSolution { };
    left_solution.input.source   = source;
    left_solution.input.in_right = false;
    left_solution.solve();

    auto armors = std::array<Armor3D, 5> { };
    armors[0]   = source;

    auto fill_neighbor = [&](Armor3D& armor, const NeighborBarSolution::Result::Bar& bar,
                             double yaw, bool in_right, int id) {
        auto rotation =
            euler_to_quaternion(yaw, kPredictedOutpostArmorPitch, 0.0).toRotationMatrix();
        auto lateral = Eigen::Vector3d { rotation * Eigen::Vector3d { 0.0, 1.0, 0.0 } };

        auto lightbar_center = Eigen::Vector3d { 0.5 * (bar.first.x + bar.second.x),
            0.5 * (bar.first.y + bar.second.y), 0.5 * (bar.first.z + bar.second.z) };
        auto center = lightbar_center - (in_right ? 1.0 : -1.0) * 0.5 * kSmallArmorWidth * lateral;

        armor.genre       = DeviceId::OUTPOST;
        armor.color       = ArmorColor::BLUE;
        armor.id          = id;
        armor.translation = Translation { center };
        armor.orientation =
            Orientation { euler_to_quaternion(yaw, kPredictedOutpostArmorPitch, 0.0) };
    };

    fill_neighbor(armors[1], right_solution.result.bars[0],
        source_yaw + 2.0 * std::numbers::pi / 3.0, true, 1);
    fill_neighbor(armors[2], right_solution.result.bars[1],
        source_yaw + 2.0 * std::numbers::pi / 3.0, true, 2);
    fill_neighbor(armors[3], left_solution.result.bars[0],
        source_yaw - 2.0 * std::numbers::pi / 3.0, false, 3);
    fill_neighbor(armors[4], left_solution.result.bars[1],
        source_yaw - 2.0 * std::numbers::pi / 3.0, false, 4);

    std::println("Publishing to /rmcs/auto_aim/outpost_armor");

    auto visuals = std::array<std::unique_ptr<Armor>, 5> { };
    auto config  = Armor::Config {
        .rclcpp = visual,
        .device = DeviceId::OUTPOST,
        .camp   = CampColor::RED,
        .id     = 0,
        .name   = "outpost_armor",
        .tf     = "odom",
    };
    for (int i = 0; i < static_cast<int>(visuals.size()); ++i) {
        config.id   = i;
        config.camp = i == 0 ? CampColor::RED : CampColor::BLUE;
        visuals[i]  = std::make_unique<Armor>(config);
    }

    while (rclcpp::ok()) {
        visual.spin_once();

        for (std::size_t i = 0; i < armors.size(); ++i) {
            visuals[i]->move(armors[i].translation, armors[i].orientation);
            visuals[i]->update();
        }

        rclcpp::sleep_for(50ms);
    }

    RclcppNode::shutdown();
    return 0;
}
