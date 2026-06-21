#include "utility/math/outpost.hpp"
#include "utility/robot/constant.hpp"

#include <eigen3/Eigen/Geometry>

namespace rmcs::util {

namespace details {

    using ArmorLevel = OutpostSolution::ArmorLevel;

    constexpr auto get_transform(ArmorLevel from, ArmorLevel to) -> std::tuple<double, double> {
        constexpr auto step = kOutpostArmorHeightStep;
        constexpr auto turn = 2.0 * std::numbers::pi / 3.0;

        if (from == to) return { 0.0, 0.0 };

        if (from == ArmorLevel::UPPER && to == ArmorLevel::MIDDLE) return { -1 * step, +turn };
        if (from == ArmorLevel::MIDDLE && to == ArmorLevel::LOWER) return { -1 * step, +turn };
        if (from == ArmorLevel::LOWER && to == ArmorLevel::UPPER) return { +2. * step, +turn };

        if (from == ArmorLevel::UPPER && to == ArmorLevel::LOWER) return { -2. * step, -turn };
        if (from == ArmorLevel::MIDDLE && to == ArmorLevel::UPPER) return { +1 * step, -turn };
        if (from == ArmorLevel::LOWER && to == ArmorLevel::MIDDLE) return { +1 * step, -turn };

        return { 0.0, 0.0 };
    }

    constexpr auto get_level(bool is_right, bool is_upper) -> std::tuple<ArmorLevel, ArmorLevel> {
        if (is_right && is_upper) return { ArmorLevel::LOWER, ArmorLevel::UPPER };
        if (is_right && !is_upper) return { ArmorLevel::UPPER, ArmorLevel::MIDDLE };
        if (!is_right && is_upper) return { ArmorLevel::MIDDLE, ArmorLevel::UPPER };
        return { ArmorLevel::UPPER, ArmorLevel::LOWER };
    }

}

auto OutpostSolution::solve() -> void {
    const auto radius = kOutpostRadius + input.armor_thickness;
    const auto pitch  = kPredictedOutpostArmorPitch;

    const auto q = input.orientation.make<Eigen::Quaterniond>();
    const auto t = input.translation.make<Eigen::Vector3d>();

    // 先求出旋转中心，按照装甲板的朝向反推即可，注意 15 度的倾角
    const auto backward = Eigen::Vector3d { q * Eigen::Vector3d::UnitX() };
    const auto armor2center =
        Eigen::Vector3d { Eigen::AngleAxisd { -pitch, q * Eigen::Vector3d::UnitY() } * backward };
    const auto rotation_center = Eigen::Vector3d { t + radius * armor2center };

    // 前哨站向上的单位向量，即水平向量绕 Y 轴朝向旋转 90 度
    const auto vertical = Eigen::Vector3d {
        Eigen::AngleAxisd { -0.5 * std::numbers::pi, q * Eigen::Vector3d::UnitY() } * armor2center
    };

    const auto [height_diff, rotate_angle] = details::get_transform(input.source, input.target);

    // 逆时针旋转为正，所以在右边需要 +，旋转 1/3 个圆
    const auto rotate2target = Eigen::AngleAxisd { rotate_angle, vertical };
    const auto target_center = Eigen::Vector3d { rotation_center
        + rotate2target * (-armor2center * radius) + vertical * height_diff };
    const auto toward        = Eigen::Vector3d { rotate2target * q * Eigen::Vector3d::UnitX() };

    const auto x_axis = Eigen::Vector3d { toward.normalized() };
    const auto y_axis = Eigen::Vector3d { vertical.cross(x_axis).normalized() };
    const auto z_axis = Eigen::Vector3d { x_axis.cross(y_axis).normalized() };

    auto rotation_matrix = Eigen::Matrix3d { };
    rotation_matrix << x_axis, y_axis, z_axis;

    result.center      = Point3d { rotation_center };
    result.translation = Translation { target_center };
    result.orientation = Orientation { Eigen::Quaterniond { rotation_matrix }.normalized() };
}

auto NeighborBarSolution::solve() -> void {
    const auto& armor = input.source;

    auto solution = OutpostSolution { };

    solution.input.translation     = armor.translation;
    solution.input.orientation     = armor.orientation;
    solution.input.armor_thickness = input.armor_thickness;

    for (const auto is_upper : { true, false }) {
        const auto [source, target] = details::get_level(input.in_right, is_upper);

        solution.input.source = source;
        solution.input.target = target;
        solution.solve();

        const auto center = solution.result.translation.make<Eigen::Vector3d>();
        const auto q      = solution.result.orientation.make<Eigen::Quaterniond>();

        const auto y_axis = Eigen::Vector3d { q * Eigen::Vector3d::UnitY() };
        const auto z_axis = Eigen::Vector3d { q * Eigen::Vector3d::UnitZ() };

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

        result.center = solution.result.center;
    }
}

}
