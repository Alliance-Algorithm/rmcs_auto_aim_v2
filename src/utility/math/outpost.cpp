#include "utility/math/outpost.hpp"

#include "utility/math/conversion.hpp"
#include "utility/robot/constant.hpp"

#include <eigen3/Eigen/Geometry>
#include <numbers>

namespace rmcs::util {

auto NeighborBarSolution::solve() -> void {
    using namespace Eigen;

    const auto source_yaw = eulers(input.source.orientation.make<Quaterniond>())[0];

    auto center = Vector3d {
        input.source.translation.x + kOutpostRadius * std::cos(source_yaw),
        input.source.translation.y + kOutpostRadius * std::sin(source_yaw),
        0.0,
    };

    const auto neighbor_yaw = input.in_right ? source_yaw + 2.0 * std::numbers::pi / 3.0
                                             : source_yaw - 2.0 * std::numbers::pi / 3.0;

    auto candidate_z = std::array<double, 2> { };
    if (input.in_right) {
        candidate_z[0] = input.source.translation.z - kOutpostArmorHeightStep;
        candidate_z[1] = input.source.translation.z + 2.0 * kOutpostArmorHeightStep;
    } else {
        candidate_z[0] = input.source.translation.z + kOutpostArmorHeightStep;
        candidate_z[1] = input.source.translation.z - 2.0 * kOutpostArmorHeightStep;
    }

    auto rotation =
        euler_to_quaternion(neighbor_yaw, kPredictedOutpostArmorPitch, 0.0).toRotationMatrix();
    const auto vertical = Eigen::Vector3d { rotation * Eigen::Vector3d { 0.0, 0.0, 1.0 } };
    const auto lateral  = Eigen::Vector3d { rotation * Eigen::Vector3d { 0.0, 1.0, 0.0 } };

    for (std::size_t i = 0; i < result.bars.size(); ++i) {
        auto neighbor_center = Eigen::Vector3d {
            center.x - kOutpostRadius * std::cos(neighbor_yaw),
            center.y - kOutpostRadius * std::sin(neighbor_yaw),
            candidate_z[i],
        };

        auto lightbar_center =
            neighbor_center + (input.in_right ? 1.0 : -1.0) * 0.5 * kSmallArmorWidth * lateral;

        const auto top    = lightbar_center + 0.5 * kLightBarHeight * vertical;
        const auto bottom = lightbar_center - 0.5 * kLightBarHeight * vertical;

        result.bars[i] = std::pair { Point3d { top }, Point3d { bottom } };
    }
}

}
