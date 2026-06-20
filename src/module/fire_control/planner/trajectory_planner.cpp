#include "module/fire_control/planner/trajectory_planner.hpp"

#include <chrono>
#include <format>
#include <optional>

#include "module/fire_control/planner/mpc_solver.hpp"
#include "module/fire_control/planner/reference_builder.hpp"
#include "utility/math/angle.hpp"
#include "utility/serializable.hpp"

using namespace rmcs::fire_control;

struct TrajectoryPlanner::Impl {
    static constexpr std::chrono::milliseconds kPlanStateTimeout { 200 };

    struct Config : util::Serializable {
        double mpc_max_yaw_acc { 50.0 };
        double mpc_yaw_q_angle { 9e6 };
        double mpc_yaw_q_rate { 0.0 };
        double mpc_yaw_r_acc { 1.0 };
        double mpc_max_pitch_acc { 100.0 };
        double mpc_pitch_q_angle { 9e6 };
        double mpc_pitch_q_rate { 0.0 };
        double mpc_pitch_r_acc { 1.0 };
        int mpc_max_iter { 10 };

        constexpr static std::tuple metas {
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
            &Config::mpc_max_iter,
            "mpc_max_iter",
        };
    } config;

    ReferenceBuilder reference_builder {};
    MpcAxisSolver yaw_solver {};
    MpcAxisSolver pitch_solver {};
    std::optional<Plan> last_plan {};
    TimePoint last_plan_timestamp {};

    static auto make_continuous_yaw_reference(
        ReferenceTrajectory const& reference, double initial_yaw) -> MpcAxisTrajectory {
        auto yaw_reference =
            MpcAxisTrajectory { reference.template block<2, kMpcHorizon>(0, 0) };

        auto previous_yaw = initial_yaw;
        for (int i = 0; i < kMpcHorizon; ++i) {
            auto const yaw =
                previous_yaw + util::normalize_angle(yaw_reference(0, i) - previous_yaw);
            yaw_reference(0, i) = yaw;
            previous_yaw        = yaw;
        }

        return yaw_reference;
    }

    auto should_reset_plan_state(TimePoint timestamp) const noexcept -> bool {
        return !last_plan.has_value() || timestamp < last_plan_timestamp
            || timestamp - last_plan_timestamp > kPlanStateTimeout;
    }

    static auto make_initial_state(ReferenceTrajectory const& reference,
        std::optional<Plan> const& previous_plan,
        int angle_row, int rate_row,
        double Plan::*prev_angle, double Plan::*prev_rate) noexcept
        -> MpcAxisSolver::AngularKinematics {
        if (!previous_plan.has_value()) {
            return {
                .angle = reference(angle_row, 0),
                .rate  = reference(rate_row, 0),
            };
        }

        return {
            .angle = (*previous_plan).*prev_angle,
            .rate  = (*previous_plan).*prev_rate,
        };
    }

    auto configure_yaml(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
        auto result = config.serialize(yaml);
        if (!result.has_value()) {
            return std::unexpected { result.error() };
        }

        result = yaw_solver.initialize(MpcAxisSolver::Config {
            .max_acc  = config.mpc_max_yaw_acc,
            .q_angle  = config.mpc_yaw_q_angle,
            .q_rate   = config.mpc_yaw_q_rate,
            .r_acc    = config.mpc_yaw_r_acc,
            .max_iter = config.mpc_max_iter,
        });
        if (!result.has_value()) {
            return std::unexpected { std::format("yaw solver init failed: {}", result.error()) };
        }

        result = pitch_solver.initialize(MpcAxisSolver::Config {
            .max_acc  = config.mpc_max_pitch_acc,
            .q_angle  = config.mpc_pitch_q_angle,
            .q_rate   = config.mpc_pitch_q_rate,
            .r_acc    = config.mpc_pitch_r_acc,
            .max_iter = config.mpc_max_iter,
        });
        if (!result.has_value()) {
            return std::unexpected {
                std::format("pitch solver init failed: {}", result.error()),
            };
        }

        return {};
    }

    auto plan(predictor::Snapshot const& snapshot, TargetSolution const& raw_solution,
        TimePoint command_time, double bullet_speed, double shoot_delay)
        -> std::expected<Plan, std::string> {
        auto reference =
            reference_builder.build(snapshot, raw_solution, command_time, bullet_speed, shoot_delay);
        if (!reference.has_value()) return std::unexpected { reference.error() };

        auto previous_plan =
            should_reset_plan_state(command_time) ? std::optional<Plan> {} : last_plan;

        auto const yaw_initial =
            make_initial_state(*reference, previous_plan, 0, 1, &Plan::yaw, &Plan::yaw_rate);
        auto const yaw_reference = make_continuous_yaw_reference(*reference, yaw_initial.angle);
        auto const yaw           = yaw_solver.solve_kinematics(yaw_reference, yaw_initial);
        if (!yaw.has_value()) {
            return std::unexpected { std::format("yaw solve failed: {}", yaw.error()) };
        }

        auto const pitch_initial =
            make_initial_state(*reference, previous_plan, 2, 3, &Plan::pitch, &Plan::pitch_rate);
        auto const pitch = pitch_solver.solve_kinematics(
            MpcAxisTrajectory { reference->template block<2, kMpcHorizon>(2, 0) },
            pitch_initial);
        if (!pitch.has_value()) {
            return std::unexpected { std::format("pitch solve failed: {}", pitch.error()) };
        }

        auto result       = Plan {};
        result.yaw        = util::normalize_angle(yaw->angle);
        result.pitch      = pitch->angle;
        result.yaw_rate   = yaw->rate;
        result.pitch_rate = pitch->rate;
        result.yaw_acc    = yaw->acc;
        result.pitch_acc  = pitch->acc;
        last_plan         = result;
        last_plan_timestamp = command_time;
        return result;
    }
};

TrajectoryPlanner::TrajectoryPlanner() noexcept
    : pimpl { std::make_unique<Impl>() } { }

TrajectoryPlanner::~TrajectoryPlanner() noexcept = default;

auto TrajectoryPlanner::configure_yaml(const YAML::Node& yaml) noexcept
    -> std::expected<void, std::string> {
    return pimpl->configure_yaml(yaml);
}

auto TrajectoryPlanner::plan(predictor::Snapshot const& snapshot,
    TargetSolution const& raw_solution, TimePoint command_time, double bullet_speed,
    double shoot_delay) -> std::expected<Plan, std::string> {
    return pimpl->plan(snapshot, raw_solution, command_time, bullet_speed, shoot_delay);
}
