#include "solve_armors.hpp"
#include <eigen3/Eigen/Dense>
#include <ranges>

namespace rmcs::util {

constexpr auto generate_corners(double w, double h) {
    using T = Eigen::Vector3d;
    using R = Eigen::Quaterniond;

    return std::array {
        std::pair { T { +0.5 * w, +0.0 * h, 0.0 },
            R { Eigen::AngleAxisd(+0.0 * std::numbers::pi, T::UnitZ()) } },

        std::pair { T { +0.0 * w, +0.5 * h, 0.0 },
            R { Eigen::AngleAxisd(+0.5 * std::numbers::pi, T::UnitZ()) } },

        std::pair { T { -0.5 * w, -0.0 * h, 0.0 },
            R { Eigen::AngleAxisd(+1.0 * std::numbers::pi, T::UnitZ()) } },

        std::pair { T { -0.0 * w, -0.5 * h, 0.0 },
            R { Eigen::AngleAxisd(-0.5 * std::numbers::pi, T::UnitZ()) } },
    };
}

auto ArmorsForwardSolution::solve() noexcept -> void {
    auto w = input.robot_width;
    auto h = input.robot_height;

    const auto t = input.t.make<Eigen::Vector3d>();
    const auto q = input.q.make<Eigen::Quaterniond>();

    const auto corners = generate_corners(w, h);

    for (auto&& [status, local] : std::views::zip(result.armors_status, corners)) {
        const auto& [local_pos, local_rot] = local;

        auto global_translation = Eigen::Vector3d { t + q * local_pos };
        auto global_orientation = Eigen::Quaterniond { q * local_rot };

        std::get<0>(status) = global_translation;
        std::get<1>(status) = global_orientation;
    }
}

}
