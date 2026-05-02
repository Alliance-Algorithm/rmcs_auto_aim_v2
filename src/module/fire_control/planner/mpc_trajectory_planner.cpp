#include "module/fire_control/planner/mpc_trajectory_planner.hpp"

#include <format>

#include "module/fire_control/planner/tiny_mpc_axis_solver.hpp"
#include "utility/math/angle.hpp"
#include "utility/serializable.hpp"

using namespace rmcs::fire_control;

struct MpcTrajectoryPlanner::Impl {
    struct Config : util::Serializable {
        double mpc_max_yaw_acc { 50.0 };
        double mpc_yaw_q_angle { 9e6 };
        double mpc_yaw_q_rate { 0.0 };
        double mpc_yaw_r_acc { 1.0 };
        double mpc_max_pitch_acc { 100.0 };
        double mpc_pitch_q_angle { 9e6 };
        double mpc_pitch_q_rate { 0.0 };
        double mpc_pitch_r_acc { 1.0 };

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
        };
    } config;

    TinyMpcAxisSolver yaw_solver {};
    TinyMpcAxisSolver pitch_solver {};

    auto configure_yaml(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
        auto result = config.serialize(yaml);
        if (!result.has_value()) {
            return std::unexpected { result.error() };
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

    auto plan(ReferenceTrajectory const& reference) -> std::expected<Plan, std::string> {
        auto const yaw = yaw_solver.solve_center_kinematics(
            MpcAxisTrajectory { reference.template block<2, kMpcAxisHorizon>(0, 0) });
        if (!yaw.has_value()) {
            return std::unexpected { std::format("yaw solve failed: {}", yaw.error()) };
        }

        auto const pitch = pitch_solver.solve_center_kinematics(
            MpcAxisTrajectory { reference.template block<2, kMpcAxisHorizon>(2, 0) });
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
        return result;
    }
};

MpcTrajectoryPlanner::MpcTrajectoryPlanner() noexcept
    : pimpl { std::make_unique<Impl>() } { }

MpcTrajectoryPlanner::~MpcTrajectoryPlanner() noexcept = default;

auto MpcTrajectoryPlanner::configure_yaml(const YAML::Node& yaml) noexcept
    -> std::expected<void, std::string> {
    return pimpl->configure_yaml(yaml);
}

auto MpcTrajectoryPlanner::initialize(const YAML::Node& yaml) noexcept
    -> std::expected<void, std::string> {
    return pimpl->configure_yaml(yaml);
}

auto MpcTrajectoryPlanner::plan(ReferenceTrajectory const& reference)
    -> std::expected<Plan, std::string> {
    return pimpl->plan(reference);
}
