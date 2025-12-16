#include "utility/acsii_art.hpp"
#include "utility/math/solve_armors.hpp"
#include "utility/rclcpp/visual/armor.hpp"
#include "utility/rclcpp/visual/posture.hpp"

#include <eigen3/Eigen/Dense>
#include <print>
#include <ranges>

#include <rclcpp/utilities.hpp>

using namespace rmcs;
using namespace rmcs::util;
using namespace rmcs::util::visual;

/**
 * @brief 启动 ROS2 可视化演示，周期性模拟并发布装甲目标与姿态的状态用于观察。
 *
 * 程序初始化 rclcpp 节点和可视化发布前缀，创建多个 Armor 实例与一个 Posture 实例，
 * 然后在主循环中根据时间驱动的平移与旋转输入更新求解器并发布各对象的状态。
 *
 * @return int 进程退出码（正常结束返回 0）。
 */
auto main() -> int {
    for (auto line : ascii_banner) {
        std::println("\033[32m{}\033[0m", line);
    }
    std::println("> Visualization Here using Rclcpp!!");
    std::println("> Use 'ros2 topic list' to check, and open foxglove to watch armors");
    std::println("> Rclcpp Prefix: {}", "/rmcs/auto_aim/");

    constexpr auto translation_speed = 3.;   // m/s
    constexpr auto orientation_speed = 6.28; // rad/s

    rclcpp::init(0, nullptr);

    auto visual = RclcppNode { "example" };
    visual.set_pub_topic_prefix("/rmcs/auto_aim/");

    auto armors = std::array<std::unique_ptr<Armor>, 4> {};
    {
        auto config = Armor::Config {
            .rclcpp = visual,
            .device = DeviceId::SENTRY,
            .camp   = CampColor::BLUE,
            .id     = 0,
            .name   = "visual_test_armor",
            .tf     = "camera_link",
        };

        for (auto i = 0; i < (int)armors.size(); ++i) {
            config.id = i;
            armors[i] = std::make_unique<visual::Armor>(config);
        }
    }

    auto posture = std::make_unique<Posture>( //
        Posture::Config {
            .rclcpp = visual,
            .id     = "sentry/posture",
            .tf     = "camera_link",
        });

    auto solution = ArmorsForwardSolution {};

    auto start = std::chrono::steady_clock::now();

    while (rclcpp::ok()) {
        const auto now = std::chrono::steady_clock::now();
        const auto t   = std::chrono::duration<double>(now - start).count();

        const auto quaternion = Eigen::Quaterniond { Eigen::AngleAxisd {
            t * orientation_speed, Eigen::Vector3d::UnitZ() } };

        solution.input.t = std::sin(translation_speed * t) * Eigen::Vector3d::UnitX();
        solution.input.q = quaternion;
        solution.solve();

        posture->move(solution.input.t, Eigen::Quaterniond::Identity());
        posture->update();

        const auto& armors_status = solution.result.armors_status;
        for (auto&& [armor, status] : std::views::zip(armors, armors_status)) {
            armor->move(status);
            armor->update();
        }

        rclcpp::sleep_for(std::chrono::milliseconds { 1'000 / 60 });
    }

    return 0;
}