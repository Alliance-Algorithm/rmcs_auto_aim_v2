#include "utility/math/outpost.hpp"
#include "utility/robot/constant.hpp"

#include <eigen3/Eigen/Geometry>

namespace rmcs::util {

auto NeighborBarSolution::solve() -> void {
    const auto radius = kOutpostRadius + input.armor_thickness;
    const auto pitch  = kPredictedOutpostArmorPitch;

    const auto& armor = input.source;

    // 先求出旋转中心，按照装甲板的朝向反推即可，注意 15 度的倾角
    const auto horizon_toward = Eigen::Vector3d { armor.orientation.make<Eigen::Quaterniond>()
        * Eigen::AngleAxisd { -pitch, Eigen::Vector3d::UnitY() } * Eigen::Vector3d::UnitX() }
                                    .normalized();
    const auto vertical_axis  = Eigen::Vector3d {
        (armor.orientation.make<Eigen::Quaterniond>() * Eigen::Vector3d::UnitZ()).normalized()
    };
    const auto outpost_center =
        Eigen::Vector3d { armor.translation.make<Eigen::Vector3d>() + horizon_toward * radius };

    result.center = Point3d { outpost_center };

    // 逆时针旋转为正，所以在右边需要 +，旋转 1/3 个圆
    const auto rotate_sign = input.in_right ? +1 : -1;
    const auto rotate2next =
        Eigen::AngleAxisd { rotate_sign * 2 * std::numbers::pi / 3., vertical_axis };

    const auto next_center =
        Eigen::Vector3d { outpost_center + rotate2next * (-horizon_toward * radius) };

    const auto steps = std::array {
        (input.in_right ? +2 : +1) * kOutpostArmorHeightStep,
        (input.in_right ? -1 : -2) * kOutpostArmorHeightStep,
    };
    const auto flags = std::array { true, false };
    for (auto&& [step, is_upper] : std::views::zip(steps, flags)) {
        const auto center = Eigen::Vector3d { next_center + vertical_axis * step };
        const auto toward = Eigen::Vector3d { rotate2next
            * armor.orientation.make<Eigen::Quaterniond>() * Eigen::Vector3d::UnitX() };

        const auto x_axis = Eigen::Vector3d { toward.normalized() };
        const auto y_axis = Eigen::Vector3d { vertical_axis.cross(x_axis).normalized() };
        const auto z_axis = Eigen::Vector3d { x_axis.cross(y_axis).normalized() };

        const auto y_step = 0.5 * kSmallArmorWidth;
        const auto z_step = 0.5 * kLightBarHeight;

        // 面对 Armor，右边的灯条
        const auto rside = Lightbar3d {
            .color = armor.color,
            .upper = Point3d { center + (y_axis * y_step) + (z_axis * z_step) },
            .lower = Point3d { center + (y_axis * y_step) - (z_axis * z_step) },
        };
        // 面对 Armor，左边的灯条
        const auto lside = Lightbar3d {
            .color = armor.color,
            .upper = Point3d { center - (y_axis * y_step) + (z_axis * z_step) },
            .lower = Point3d { center - (y_axis * y_step) - (z_axis * z_step) },
        };

        // 根据实际距离选取靠近的灯条
        const auto o = armor.translation.make<Eigen::Vector3d>();

        const auto lside_dist = (lside.upper.make<Eigen::Vector3d>() - o).norm();
        const auto rside_dist = (rside.upper.make<Eigen::Vector3d>() - o).norm();

        const auto& near = lside_dist < rside_dist ? lside : rside;
        const auto& away = lside_dist < rside_dist ? rside : lside;

        // 根据上下侧来选定赋值目标
        (is_upper ? result.upper_near : result.lower_near) = near;
        (is_upper ? result.upper_away : result.lower_away) = away;
    }
}

}
