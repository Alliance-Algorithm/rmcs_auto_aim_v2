#include "solve_armors.hpp"
#include <eigen3/Eigen/Dense>
#include <ranges>

namespace rmcs::util {

constexpr auto generate_corners(double w, double h) {
    return std::array {
        Eigen::Vector3d { +0.5 * w, +0.0 * h, 0.0 },
        Eigen::Vector3d { +0.0 * w, +0.5 * h, 0.0 },
        Eigen::Vector3d { -0.5 * w, -0.0 * h, 0.0 },
        Eigen::Vector3d { -0.0 * w, -0.5 * h, 0.0 },
    };
}

auto ArmorsForwardSolution::solve() noexcept -> void {
    auto w = input.robot_width;
    auto h = input.robot_height;

    const auto t = input.t.make<Eigen::Vector3d>();
    const auto q = input.q.make<Eigen::Quaterniond>();

    const auto corners = generate_corners(w, h);
    for (auto&& [status, corner] : std::views::zip(result.armors_status, corners)) {
        const auto global_translation = Eigen::Vector3d { q * corner + t };

        auto point_to_center = Eigen::Vector3d { t - global_translation };
        point_to_center.normalize();

        const auto global_orientation =
            Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), point_to_center);

        std::get<0>(status) = global_translation;
        std::get<1>(status) = global_orientation;
    }
}

}
