#include "utility/rclcpp/visualization.hpp"
#include "utility/math/solve_armors.hpp"
#include <eigen3/Eigen/Dense>
#include <print>
#include <rclcpp/utilities.hpp>

auto main() -> int {
    using namespace rmcs;
    using namespace rmcs::util;

    constexpr auto translation_speed = 3.;   // m
    constexpr auto orientation_speed = 6.28; // rad

    rclcpp::init(0, nullptr);

    std::println("> Hello World!!!");
    std::println("> Visualization Here using Rclcpp");

    auto visual = VisualNode { "example" };

    auto armors = std::array<std::unique_ptr<visual::Armor>, 4> {};
    auto index  = char { 'a' };
    std::ranges::for_each(armors, [&](auto& armor) {
        const auto naming { std::string { "sentry_" } + index++ };
        armor = visual.make<visual::Armor>(naming, "camera_link");
        armor->init(DeviceId::SENTRY, CampColor::BLUE);
    });

    auto solution = ArmorsForwardSolution {};

    auto start = std::chrono::steady_clock::now();

    while (rclcpp::ok()) {
        const auto now = std::chrono::steady_clock::now();
        const auto t   = std::chrono::duration<double>(now - start).count();

        const auto angle_axisd =
            Eigen::AngleAxisd { t * orientation_speed, Eigen::Vector3d::UnitZ() };
        const auto quaternion = Eigen::Quaterniond { angle_axisd };

        solution.input.t = std::sin(translation_speed * t) * Eigen::Vector3d::UnitX();
        solution.input.q = quaternion;
        solution.solve();

        const auto& armors_status = solution.result.armors_status;
        for (auto&& [armor, status] : std::views::zip(armors, armors_status)) {
            armor->set_translation(std::get<0>(status));
            armor->set_orientation(std::get<1>(status));
            armor->update();
        }

        rclcpp::sleep_for(std::chrono::milliseconds { 1'000 / 60 });
    }

    return 0;
}
