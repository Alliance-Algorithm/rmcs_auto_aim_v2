#include "module/fire_control/solver/aim_point_sampling.hpp"

#include <cmath>

#include "module/fire_control/trajectory_solution.hpp"
#include "utility/math/angle.hpp"

using namespace rmcs::fire_control;

auto rmcs::fire_control::AimPointSampler::sample_at(
    predictor::Snapshot const& snapshot, AimPointChooser& chooser, TimePoint t, double bullet_speed)
    -> std::expected<AimSample, std::string> {
    auto aim_point = sample_aim_point_at(snapshot, chooser, t);
    if (!aim_point.has_value()) {
        return std::unexpected { aim_point.error() };
    }

    auto attitude = solve_aim_attitude(aim_point.value(), bullet_speed);
    if (!attitude.has_value()) {
        return std::unexpected { attitude.error() };
    }

    return AimSample {
        .attitude  = *attitude,
        .aim_point = aim_point.value(),
    };
}

auto rmcs::fire_control::AimPointSampler::sample_aim_point_at(predictor::Snapshot const& snapshot,
    AimPointChooser& chooser, TimePoint t) -> std::expected<Eigen::Vector3d, std::string> {
    auto predicted_armors     = snapshot.predicted_armors(t);
    auto predicted_kinematics = snapshot.kinematics_at(t);
    auto chosen_armor = chooser.choose_armor(predicted_armors, predicted_kinematics.center_position,
        predicted_kinematics.angular_velocity);
    if (!chosen_armor.has_value()) return std::unexpected { "choose_armor returned nullopt" };

    auto aim_point = Eigen::Vector3d {};
    chosen_armor->translation.copy_to(aim_point);
    return aim_point;
}

auto rmcs::fire_control::AimPointSampler::solve_aim_attitude(Eigen::Vector3d const& aim_point,
    double bullet_speed) -> std::expected<AimAttitude, std::string> {
    auto const target_d = std::hypot(aim_point.x(), aim_point.y());
    if (!(target_d > 0.0)) {
        return std::unexpected { "invalid target distance" };
    }

    auto solution                  = TrajectorySolution {};
    solution.input.v0              = bullet_speed;
    solution.input.target_position = aim_point;

    auto trajectory = solution.solve();
    if (!trajectory.has_value()) {
        return std::unexpected { "trajectory solve failed" };
    }

    return AimAttitude {
        .yaw      = util::normalize_angle(trajectory->yaw),
        .pitch    = trajectory->pitch,
        .fly_time = trajectory->fly_time,
    };
}
