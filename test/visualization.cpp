#include "utility/acsii_art.hpp"
#include "utility/math/solve_armors.hpp"
#include "utility/rclcpp/visual/armor.hpp"
#include <eigen3/Eigen/Dense>
#include <print>
#include <ranges>
#include <rclcpp/utilities.hpp>

auto main() -> int {
    using namespace rmcs;
    using namespace rmcs::util;

    for (auto line : ascii_banner) {
        std::println("\033[32m{}\033[0m", line);
    }
    std::println("> Visualization Here using Rclcpp!!");
    std::println("> Use 'ros2 topic list' to check, and open foxglove to watch armors");
    std::println("> Rclcpp Prefix: {}", "/rmcs/auto_aim/");

    constexpr auto translation_speed = 3.;   // m
    constexpr auto orientation_speed = 6.28; // rad

    rclcpp::init(0, nullptr);

    auto visual = RclcppNode { "example" };

    auto config = visual::Armor::Config {
        .rclcpp = visual,
        .device = DeviceId::SENTRY,
        .camp   = CampColor::BLUE,
        .id     = "",
        .tf     = "camera_link",
    };

    auto armors = std::array<std::unique_ptr<visual::Armor>, 4> {};
    auto index  = char { 'a' };
    std::ranges::for_each(armors, [&](auto& armor) {
        config.id = std::string { "sentry_" } + index++;
        armor     = std::make_unique<visual::Armor>(config);
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
            armor->move(status);
            armor->update();
        }

        rclcpp::sleep_for(std::chrono::milliseconds { 1'000 / 60 });
    }

    return 0;
}
