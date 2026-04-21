#include "mpc_trajectory_planner.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <format>

#include <tinympc/tiny_api.hpp>

#include "module/fire_control/trajectory_solution.hpp"
#include "utility/math/angle.hpp"
#include "utility/serializable.hpp"

using namespace rmcs::fire_control;

namespace {

constexpr double kMpcDt    = 0.01;
constexpr int kHalfHorizon = 50;
constexpr int kHorizon     = kHalfHorizon * 2;

using ReferenceTrajectory = Eigen::Matrix<double, 4, kHorizon>;

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

auto to_duration(double seconds) -> rmcs::predictor::Snapshot::Clock::duration {
    return std::chrono::duration_cast<rmcs::predictor::Snapshot::Clock::duration>(
        std::chrono::duration<double> { seconds });
}

auto destroy_solver(TinySolver*& solver) noexcept -> void {
    if (solver == nullptr) return;
    delete solver->solution;
    delete solver->cache;
    delete solver->settings;
    delete solver->work;
    delete solver;
    solver = nullptr;
}

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
    };

    Config config {};
    AimPointChooser::Config chooser_config {};

    TinySolver* yaw_solver { nullptr };
    TinySolver* pitch_solver { nullptr };

    ~Impl() noexcept {
        destroy_solver(yaw_solver);
        destroy_solver(pitch_solver);
    }

    auto initialize(const YAML::Node& yaml, AimPointChooser::Config const& config_in) noexcept
        -> std::expected<void, std::string> {
        chooser_config = config_in;

        auto result = config.serialize(yaml);
        if (!result.has_value()) {
            return std::unexpected { result.error() };
        }

        if (!config.mpc_enable) return {};

        result = setup_solver(yaw_solver, config.mpc_max_yaw_acc, config.mpc_yaw_q_angle,
            config.mpc_yaw_q_rate, config.mpc_yaw_r_acc);
        if (!result.has_value()) {
            return std::unexpected { std::format("yaw solver init failed: {}", result.error()) };
        }

        result = setup_solver(pitch_solver, config.mpc_max_pitch_acc, config.mpc_pitch_q_angle,
            config.mpc_pitch_q_rate, config.mpc_pitch_r_acc);
        if (!result.has_value()) {
            return std::unexpected {
                std::format("pitch solver init failed: {}", result.error()),
            };
        }

        return {};
    }

    auto plan(const predictor::Snapshot& snapshot,
        predictor::Snapshot::Clock::time_point center_time, double bullet_speed, double yaw_offset,
        double pitch_offset) -> std::optional<Plan> {
        if (!config.mpc_enable || yaw_solver == nullptr || pitch_solver == nullptr) {
            return std::nullopt;
        }

        auto reference =
            generate_reference(snapshot, center_time, bullet_speed, yaw_offset, pitch_offset);
        if (!reference) return std::nullopt;

        auto yaw_x0 = Eigen::Vector2d {};
        yaw_x0 << reference->trajectory(0, 0), reference->trajectory(1, 0);
        if (tiny_set_x0(yaw_solver, yaw_x0) != 0) return std::nullopt;

        auto yaw_ref = Eigen::MatrixXd { reference->trajectory.block(0, 0, 2, kHorizon) };
        if (tiny_set_x_ref(yaw_solver, yaw_ref) != 0) return std::nullopt;
        if (tiny_solve(yaw_solver) != 0) return std::nullopt;

        auto pitch_x0 = Eigen::Vector2d {};
        pitch_x0 << reference->trajectory(2, 0), reference->trajectory(3, 0);
        if (tiny_set_x0(pitch_solver, pitch_x0) != 0) return std::nullopt;

        auto pitch_ref = Eigen::MatrixXd { reference->trajectory.block(2, 0, 2, kHorizon) };
        if (tiny_set_x_ref(pitch_solver, pitch_ref) != 0) return std::nullopt;
        if (tiny_solve(pitch_solver) != 0) return std::nullopt;

        auto result         = Plan {};
        result.target_yaw   = reference->target_yaw;
        result.target_pitch = reference->target_pitch;
        result.yaw =
            util::normalize_angle(yaw_solver->work->x(0, kHalfHorizon) + reference->yaw_origin);
        result.pitch = pitch_solver->work->x(0, kHalfHorizon);
        return result;
    }

private:
    static auto setup_solver(TinySolver*& solver, double max_acc, double q_angle, double q_rate,
        double r_acc) -> std::expected<void, std::string> {
        destroy_solver(solver);

        auto A = Eigen::MatrixXd(2, 2);
        A << 1.0, kMpcDt, 0.0, 1.0;
        auto B = Eigen::MatrixXd(2, 1);
        B << 0.0, kMpcDt;
        auto f = Eigen::MatrixXd(2, 1);
        f << 0.0, 0.0;

        Eigen::Matrix2d Q = Eigen::Matrix2d::Zero();
        Q(0, 0)           = q_angle;
        Q(1, 1)           = q_rate;
        Eigen::Matrix<double, 1, 1> R;
        R << r_acc;

        if (tiny_setup(&solver, A, B, f, Q, R, 1.0, 2, 1, kHorizon, 0) != 0 || solver == nullptr) {
            return std::unexpected { "tiny_setup returned non-zero status" };
        }

        auto x_min = Eigen::MatrixXd::Constant(2, kHorizon, -1e17);
        auto x_max = Eigen::MatrixXd::Constant(2, kHorizon, 1e17);
        auto u_min = Eigen::MatrixXd::Constant(1, kHorizon - 1, -max_acc);
        auto u_max = Eigen::MatrixXd::Constant(1, kHorizon - 1, max_acc);
        if (tiny_set_bound_constraints(solver, x_min, x_max, u_min, u_max) != 0) {
            destroy_solver(solver);
            return std::unexpected { "tiny_set_bound_constraints returned non-zero status" };
        }

        solver->settings->max_iter = 10;
        return {};
    }

    static auto sample_at(const predictor::Snapshot& snapshot,
        predictor::Snapshot::Clock::time_point t, double bullet_speed, double yaw_offset,
        double pitch_offset, AimPointChooser& chooser) -> std::optional<AimSample> {
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

    auto generate_reference(const predictor::Snapshot& snapshot,
        predictor::Snapshot::Clock::time_point center_time, double bullet_speed, double yaw_offset,
        double pitch_offset) const -> std::optional<ReferencePlan> {
        auto chooser = AimPointChooser {};
        chooser.initialize(chooser_config);

        auto samples = std::array<AimSample, kHorizon + 2> {};
        for (int i = 0; i < static_cast<int>(samples.size()); ++i) {
            auto const offset = (i - (kHalfHorizon + 1)) * kMpcDt;
            auto const t      = center_time + to_duration(offset);
            auto sample = sample_at(snapshot, t, bullet_speed, yaw_offset, pitch_offset, chooser);
            if (!sample) return std::nullopt;
            samples[i] = *sample;
        }

        auto const yaw_origin   = samples[kHalfHorizon + 1].yaw;
        auto const target_yaw   = yaw_origin;
        auto const target_pitch = samples[kHalfHorizon + 1].pitch;

        auto trajectory = ReferenceTrajectory {};
        for (int i = 0; i < kHorizon; ++i) {
            auto const& previous = samples[i];
            auto const& current  = samples[i + 1];
            auto const& next     = samples[i + 2];

            auto const yaw_velocity =
                util::normalize_angle(next.yaw - previous.yaw) / (2.0 * kMpcDt);
            auto const pitch_velocity = (next.pitch - previous.pitch) / (2.0 * kMpcDt);

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

auto MpcTrajectoryPlanner::initialize(const YAML::Node& yaml,
    AimPointChooser::Config const& chooser_config) noexcept -> std::expected<void, std::string> {
    return pimpl->initialize(yaml, chooser_config);
}

auto MpcTrajectoryPlanner::plan(const predictor::Snapshot& snapshot,
    predictor::Snapshot::Clock::time_point center_time, double bullet_speed, double yaw_offset,
    double pitch_offset) -> std::optional<Plan> {
    return pimpl->plan(snapshot, center_time, bullet_speed, yaw_offset, pitch_offset);
}
