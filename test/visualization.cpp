#include "utility/rclcpp/visualization.hpp"
#include <eigen3/Eigen/Dense>
#include <print>
#include <rclcpp/rclcpp.hpp>

auto main() -> int {
    using namespace rmcs;
    using namespace rmcs::util;

    rclcpp::init(0, nullptr);

    std::println("> Hello World!!!");
    std::println("> Visualization Here using Rclcpp");

    auto visualization = Visualization { "example" };

    auto armor_a = visualization.make_visualized<visual::Armor>("sentry_a", "camera_link");
    auto armor_b = visualization.make_visualized<visual::Armor>("sentry_b", "camera_link");
    auto armor_c = visualization.make_visualized<visual::Armor>("sentry_c", "camera_link");
    auto armor_d = visualization.make_visualized<visual::Armor>("sentry_d", "camera_link");
    armor_a->init(DeviceId::INFANTRY_3, CampColor::RED);
    armor_b->init(DeviceId::INFANTRY_3, CampColor::RED);
    armor_c->init(DeviceId::INFANTRY_3, CampColor::RED);
    armor_d->init(DeviceId::INFANTRY_3, CampColor::RED);

    auto start = std::chrono::steady_clock::now();

    while (rclcpp::ok()) {
        auto now = std::chrono::steady_clock::now();
        double t = std::chrono::duration<double>(now - start).count();

        // 整体绕 z 轴旋转：角速度 1 rad/s
        double angle = t * 3;
        Eigen::AngleAxisd rot(angle, Eigen::Vector3d::UnitZ());

        // 四个角点（正方形边长 0.5 m）
        double half                            = 0.25;
        std::array<Eigen::Vector3d, 4> corners = { Eigen::Vector3d(half, half, 0.0),
            Eigen::Vector3d(-half, half, 0.0), Eigen::Vector3d(-half, -half, 0.0),
            Eigen::Vector3d(half, -half, 0.0) };

        // 更新四个装甲板
        auto update_armor = [&](const auto& armor, const Eigen::Vector3d& corner) {
            Eigen::Vector3d pos = rot * corner;
            armor->set_translation(pos);

            // 让装甲板朝向中心 (0,0,z)
            Eigen::Vector3d to_center = pos;
            to_center.normalize();

            // 假设装甲板的局部 x 轴是“正前方”
            auto orient = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), to_center);
            armor->set_orientation(orient);

            armor->update();
        };

        update_armor(armor_a, corners[0]);
        update_armor(armor_b, corners[1]);
        update_armor(armor_c, corners[2]);
        update_armor(armor_d, corners[3]);

        using namespace std::chrono_literals;
        rclcpp::sleep_for(5ms);
    }

    return 0;
}
