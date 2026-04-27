#include "module/fire_control/planner/mpc_trajectory_planner.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <format>

#include "module/fire_control/aim_point_chooser.hpp"
#include "module/fire_control/planner/tiny_mpc_axis_solver.hpp"
#include "module/fire_control/trajectory_solution.hpp"
#include "utility/math/angle.hpp"

using namespace rmcs::fire_control;

namespace {

constexpr double kMpcAxisDt       = 0.01;
constexpr int kMpcAxisHalfHorizon = kMpcAxisHorizon / 2;

using ReferenceTrajectory = Eigen::Matrix<double, 4, kMpcAxisHorizon>;

struct AimSample {
    double yaw;
    double pitch;
};

struct ReferencePlan {
    ReferenceTrajectory trajectory;
    double yaw_origin;
    double target_yaw;
    double target_pitch;
};

} // namespace

struct MpcTrajectoryPlanner::Impl {
    Config config {};
    AimPointChooser::Config chooser_config {};

    TinyMpcAxisSolver yaw_solver {};
    TinyMpcAxisSolver pitch_solver {};

    auto initialize(Config const& config_in) noexcept -> std::expected<void, std::string> {
        config         = config_in;
        chooser_config = AimPointChooser::Config {
            .coming_angle          = config.coming_angle,
            .leaving_angle         = config.leaving_angle,
            .outpost_coming_angle  = config.outpost_coming_angle,
            .outpost_leaving_angle = config.outpost_leaving_angle,
        };

        if (!config.mpc_enable) {
            return {};
        }

        auto result = yaw_solver.initialize(TinyMpcAxisSolver::Config {
            .max_acc = config.mpc_max_yaw_acc,
            .q_angle = config.mpc_yaw_q_angle,
            .q_rate  = config.mpc_yaw_q_rate,
            .r_acc   = config.mpc_yaw_r_acc,
        });
        if (!result.has_value()) {
            return std::unexpected { std::format("yaw solver init failed: {}", result.error()) };
        }

        result = pitch_solver.initialize(TinyMpcAxisSolver::Config {
            .max_acc = config.mpc_max_pitch_acc,
            .q_angle = config.mpc_pitch_q_angle,
            .q_rate  = config.mpc_pitch_q_rate,
            .r_acc   = config.mpc_pitch_r_acc,
        });
        if (!result.has_value()) {
            return std::unexpected {
                std::format("pitch solver init failed: {}", result.error()),
            };
        }

        return {};
    }

    auto plan(const predictor::Snapshot& snapshot, TimePoint center_time, double bullet_speed,
        double yaw_offset, double pitch_offset) -> std::optional<Plan> {
        if (!config.mpc_enable) {
            return std::nullopt;
        }

        auto reference =
            generate_reference(snapshot, center_time, bullet_speed, yaw_offset, pitch_offset);
        if (!reference) return std::nullopt;

        auto const yaw = yaw_solver.solve_center(
            MpcAxisTrajectory { reference->trajectory.template block<2, kMpcAxisHorizon>(0, 0) });
        if (!yaw.has_value()) return std::nullopt;

        auto const pitch = pitch_solver.solve_center(
            MpcAxisTrajectory { reference->trajectory.template block<2, kMpcAxisHorizon>(2, 0) });
        if (!pitch.has_value()) return std::nullopt;

        auto result         = Plan {};
        result.target_yaw   = reference->target_yaw;
        result.target_pitch = reference->target_pitch;
        result.yaw          = util::normalize_angle(yaw.value() + reference->yaw_origin);
        result.pitch        = pitch.value();
        return result;
    }

private:
    static auto sample_at(const predictor::Snapshot& snapshot, TimePoint t, double bullet_speed,
        double yaw_offset, double pitch_offset, AimPointChooser& chooser)
        -> std::optional<AimSample> {
        auto predicted_armors     = snapshot.predicted_armors(t);
        auto predicted_kinematics = snapshot.kinematics_at(t);

        auto chosen = chooser.choose_armor(predicted_armors, predicted_kinematics.center_position,
            predicted_kinematics.angular_velocity);
        if (!chosen) return std::nullopt;

        auto armor_position = Eigen::Vector3d {};
        chosen->translation.copy_to(armor_position);

        auto const target_d = std::hypot(armor_position.x(), armor_position.y());
        if (!(target_d > 0.0)) return std::nullopt;

        auto solution           = TrajectorySolution {};
        solution.input.v0       = bullet_speed;
        solution.input.target_d = target_d;
        solution.input.target_h = armor_position.z();

        auto trajectory = solution.solve();
        if (!trajectory) return std::nullopt;

        return AimSample {
            .yaw = util::normalize_angle(
                std::atan2(armor_position.y(), armor_position.x()) + yaw_offset),
            .pitch = trajectory->pitch + pitch_offset,
        };
    }

    auto generate_reference(const predictor::Snapshot& snapshot, TimePoint center_time,
        double bullet_speed, double yaw_offset, double pitch_offset) const
        -> std::optional<ReferencePlan> {
        auto chooser = AimPointChooser {};
        chooser.initialize(chooser_config);

        auto samples = std::array<AimSample, kMpcAxisHorizon + 2> {};
        for (int i = 0; i < static_cast<int>(samples.size()); ++i) {
            auto const offset = (i - (kMpcAxisHalfHorizon + 1)) * kMpcAxisDt;
            auto const t      = center_time
                + std::chrono::duration_cast<rmcs::Duration>(
                    std::chrono::duration<double> { offset });
            auto sample = sample_at(snapshot, t, bullet_speed, yaw_offset, pitch_offset, chooser);
            if (!sample) return std::nullopt;
            samples[i] = *sample;
        }

        auto const yaw_origin   = samples[kMpcAxisHalfHorizon + 1].yaw;
        auto const target_yaw   = yaw_origin;
        auto const target_pitch = samples[kMpcAxisHalfHorizon + 1].pitch;

        auto trajectory = ReferenceTrajectory {};
        for (int i = 0; i < kMpcAxisHorizon; ++i) {
            auto const& previous = samples[i];
            auto const& current  = samples[i + 1];
            auto const& next     = samples[i + 2];

            auto const yaw_velocity =
                util::normalize_angle(next.yaw - previous.yaw) / (2.0 * kMpcAxisDt);
            auto const pitch_velocity = (next.pitch - previous.pitch) / (2.0 * kMpcAxisDt);

            trajectory.col(i) << util::normalize_angle(current.yaw - yaw_origin), yaw_velocity,
                current.pitch, pitch_velocity;
        }

        return ReferencePlan {
            .trajectory   = trajectory,
            .yaw_origin   = yaw_origin,
            .target_yaw   = target_yaw,
            .target_pitch = target_pitch,
        };
    }
};

MpcTrajectoryPlanner::MpcTrajectoryPlanner() noexcept
    : pimpl { std::make_unique<Impl>() } { }

MpcTrajectoryPlanner::~MpcTrajectoryPlanner() noexcept = default;

auto MpcTrajectoryPlanner::initialize(Config const& config) noexcept
    -> std::expected<void, std::string> {
    return pimpl->initialize(config);
}

auto MpcTrajectoryPlanner::plan(const predictor::Snapshot& snapshot, TimePoint center_time,
    double bullet_speed, double yaw_offset, double pitch_offset) -> std::optional<Plan> {
    return pimpl->plan(snapshot, center_time, bullet_speed, yaw_offset, pitch_offset);
}
