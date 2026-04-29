#include "module/fire_control/planner/mpc_trajectory_planner.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <format>

#include "module/fire_control/planner/tiny_mpc_axis_solver.hpp"
#include "module/fire_control/trajectory_solution.hpp"
#include "utility/math/angle.hpp"
#include "utility/serializable.hpp"

using namespace rmcs::fire_control;

namespace {

constexpr double kMpcAxisDt       = 0.01;
constexpr int kMpcAxisHalfHorizon = kMpcAxisHorizon / 2;

// ReferenceTrajectory row layout (4 x kMpcAxisHorizon):
// row 0: yaw relative to yaw_origin (rad)
// row 1: yaw angular velocity (rad/s)
// row 2: pitch angle (rad)
// row 3: pitch angular velocity (rad/s)
using ReferenceTrajectory = Eigen::Matrix<double, 4, kMpcAxisHorizon>;

struct AimSample {
    double yaw;
    double pitch;
};

struct ReferencePlan {
    ReferenceTrajectory trajectory;
    double target_yaw;
    double target_pitch;
};

} // namespace

struct MpcTrajectoryPlanner::Impl {
    struct Config : util::Serializable {
        bool mpc_enable { true };
        double mpc_max_yaw_acc { 50.0 };
        double mpc_yaw_q_angle { 9e6 };
        double mpc_yaw_q_rate { 0.0 };
        double mpc_yaw_r_acc { 1.0 };
        double mpc_max_pitch_acc { 100.0 };
        double mpc_pitch_q_angle { 9e6 };
        double mpc_pitch_q_rate { 0.0 };
        double mpc_pitch_r_acc { 1.0 };

        constexpr static std::tuple metas {
            &Config::mpc_enable,
            "mpc_enable",
            &Config::mpc_max_yaw_acc,
            "mpc_max_yaw_acc",
            &Config::mpc_yaw_q_angle,
            "mpc_yaw_q_angle",
            &Config::mpc_yaw_q_rate,
            "mpc_yaw_q_rate",
            &Config::mpc_yaw_r_acc,
            "mpc_yaw_r_acc",
            &Config::mpc_max_pitch_acc,
            "mpc_max_pitch_acc",
            &Config::mpc_pitch_q_angle,
            "mpc_pitch_q_angle",
            &Config::mpc_pitch_q_rate,
            "mpc_pitch_q_rate",
            &Config::mpc_pitch_r_acc,
            "mpc_pitch_r_acc",
        };
    } config;

    TinyMpcAxisSolver yaw_solver {};
    TinyMpcAxisSolver pitch_solver {};

    auto configure_yaml(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
        auto result = config.serialize(yaml);
        if (!result.has_value()) {
            return std::unexpected { result.error() };
        }

        // TODO:
        if (!config.mpc_enable) {
            return {};
        }

        result = yaw_solver.initialize(TinyMpcAxisSolver::Config {
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

    auto plan(TimePoint center_time, double bullet_speed, double yaw_offset, double pitch_offset,
        AimPointSampler const& sample_aim_point) -> std::optional<Plan> {
        if (!config.mpc_enable) return std::nullopt;

        auto reference = generate_reference(
            center_time, bullet_speed, yaw_offset, pitch_offset, sample_aim_point);
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
        result.yaw          = util::normalize_angle(yaw.value() + reference->target_yaw);
        result.pitch        = pitch.value();
        return result;
    }

private:
    static auto sample_at(TimePoint t, double bullet_speed, double yaw_offset, double pitch_offset,
        AimPointSampler const& sample_aim_point) -> std::optional<AimSample> {
        auto target = sample_aim_point(t);
        if (!target) return std::nullopt;
        auto const& aim_position = *target;

        auto const target_d = std::hypot(aim_position.x(), aim_position.y());
        if (!(target_d > 0.0)) return std::nullopt;

        auto solution           = TrajectorySolution {};
        solution.input.v0       = bullet_speed;
        solution.input.target_d = target_d;
        solution.input.target_h = aim_position.z();

        auto trajectory = solution.solve();
        if (!trajectory) return std::nullopt;

        return AimSample {
            .yaw =
                util::normalize_angle(std::atan2(aim_position.y(), aim_position.x()) + yaw_offset),
            .pitch = trajectory->pitch + pitch_offset,
        };
    }

    auto generate_reference(TimePoint center_time, double bullet_speed, double yaw_offset,
        double pitch_offset, AimPointSampler const& sample_aim_point) const
        -> std::optional<ReferencePlan> {
        constexpr int kCenterSampleIndex = kMpcAxisHalfHorizon + 1;

        auto samples = std::array<AimSample, kMpcAxisHorizon + 2> {};
        for (int i = 0; i < static_cast<int>(samples.size()); ++i) {
            auto const offset = (i - kCenterSampleIndex) * kMpcAxisDt;
            auto const t      = center_time
                + std::chrono::duration_cast<rmcs::Duration>(
                    std::chrono::duration<double> { offset });
            auto sample = sample_at(t, bullet_speed, yaw_offset, pitch_offset, sample_aim_point);
            if (!sample) return std::nullopt;
            samples[i] = *sample;
        }

        auto const yaw_origin   = samples[kCenterSampleIndex].yaw;
        auto const target_yaw   = yaw_origin;
        auto const target_pitch = samples[kCenterSampleIndex].pitch;

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
            .target_yaw   = target_yaw,
            .target_pitch = target_pitch,
        };
    }
};

MpcTrajectoryPlanner::MpcTrajectoryPlanner() noexcept
    : pimpl { std::make_unique<Impl>() } { }
MpcTrajectoryPlanner::~MpcTrajectoryPlanner() noexcept = default;

auto MpcTrajectoryPlanner::configure_yaml(const YAML::Node& yaml) noexcept
    -> std::expected<void, std::string> {
    return pimpl->configure_yaml(yaml);
}

auto MpcTrajectoryPlanner::plan(TimePoint center_time, double bullet_speed, double yaw_offset,
    double pitch_offset, AimPointSampler const& sample_aim_point) -> std::optional<Plan> {
    return pimpl->plan(center_time, bullet_speed, yaw_offset, pitch_offset, sample_aim_point);
}
