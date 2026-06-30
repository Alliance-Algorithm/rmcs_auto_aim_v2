#include "robot.hpp"
#include "utility/robot/constant.hpp"

#include <eigen3/Eigen/Geometry>

using namespace rmcs;

namespace details {

struct ArmorPose {
    Eigen::Vector3d translation;
    Eigen::Quaterniond orientation;
};

auto compute_armor_poses(const RobotSolution::Input& input) {
    const auto q = input.toward.make<Eigen::Quaterniond>();
    const auto c = input.center.make<Eigen::Vector3d>();

    const auto px_dir = Eigen::Vector3d { q * Eigen::Vector3d::UnitX() };
    const auto py_dir = Eigen::Vector3d { q * Eigen::Vector3d::UnitY() };

    constexpr auto pitch = kPredictedOtherArmorPitch;

    auto make_orientation = [&q, pitch](double delta) {
        return Eigen::Quaterniond {
            Eigen::AngleAxisd { delta, Eigen::Vector3d::UnitZ() } * q
                * Eigen::AngleAxisd { pitch, Eigen::Vector3d::UnitY() },
        };
    };

    const auto fr = input.radius_forward;
    const auto lr = input.radius_lateral;
    const auto lh = input.height_lateral;
    const auto vz = Eigen::Vector3d::UnitZ();
    const auto v0 = Eigen::Vector3d::Zero();

    // id 绕 Z 轴逆时针: px=0, py=1, nx=2, ny=3
    // delta = armor→center 方向相对 toward X 轴的 Z 轴旋转量
    return std::array {
        ArmorPose { c + fr * px_dir + v0 * lh, make_orientation(std::numbers::pi * +1.0) },
        ArmorPose { c + lr * py_dir + vz * lh, make_orientation(std::numbers::pi * -0.5) },
        ArmorPose { c - fr * px_dir + v0 * lh, make_orientation(std::numbers::pi * +0.0) },
        ArmorPose { c - lr * py_dir + vz * lh, make_orientation(std::numbers::pi * +0.5) },
    };
}

} // namespace

auto RobotSolution::solve_armors() -> std::array<Armor3d, 4> {
    const auto poses = details::compute_armor_poses(input);

    auto make_armor = [&](int id, const details::ArmorPose& pose) {
        return Armor3d {
            .genre       = input.genre,
            .color       = input.color,
            .id          = id,
            .translation = Translation { pose.translation },
            .orientation = Orientation { pose.orientation },
        };
    };

    return {
        make_armor(0, poses[0]),
        make_armor(1, poses[1]),
        make_armor(2, poses[2]),
        make_armor(3, poses[3]),
    };
}

auto RobotSolution::solve_lightbars() -> std::array<Lightbar3d, 8> {
    const auto poses = details::compute_armor_poses(input);

    const auto is_large = DeviceIds::kLargeArmor().contains(input.genre);
    const auto half_w   = (is_large ? kLargeArmorWidth : kSmallArmorWidth) * 0.5;
    const auto half_h   = kLightBarHeight * 0.5;

    auto result = std::array<Lightbar3d, 8> { };

    for (const auto i : std::views::iota(0, 4)) {
        const auto y_axis = Eigen::Vector3d { poses[i].orientation * Eigen::Vector3d::UnitY() };
        const auto z_axis = Eigen::Vector3d { poses[i].orientation * Eigen::Vector3d::UnitZ() };

        // 逆时针: px=0(−Y)/1(+Y), py=2(+X)/3(−X), nx=4(+Y)/5(−Y), ny=6(−X)/7(+X)
        const auto center = Eigen::Vector3d { poses[i].translation };
        const auto ev_pt  = Eigen::Vector3d { center + y_axis * half_w };
        const auto od_pt  = Eigen::Vector3d { center - y_axis * half_w };

        const auto ev_up = Eigen::Vector3d { ev_pt + z_axis * half_h };
        const auto ev_lo = Eigen::Vector3d { ev_pt - z_axis * half_h };
        const auto od_up = Eigen::Vector3d { od_pt + z_axis * half_h };
        const auto od_lo = Eigen::Vector3d { od_pt - z_axis * half_h };

        result[i * 2 + 0] = Lightbar3d {
            .color = input.color,
            .upper = Point3d { ev_up.x(), ev_up.y(), ev_up.z() },
            .lower = Point3d { ev_lo.x(), ev_lo.y(), ev_lo.z() },
        };
        result[i * 2 + 1] = Lightbar3d {
            .color = input.color,
            .upper = Point3d { od_up.x(), od_up.y(), od_up.z() },
            .lower = Point3d { od_lo.x(), od_lo.y(), od_lo.z() },
        };
    }
    return result;
}
