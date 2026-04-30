#include "fire_control.hpp"

#include <cmath>
#include <format>
#include <memory>
#include <optional>

#include "module/fire_control/aim_point_chooser.hpp"
#include "module/fire_control/planner/mpc_trajectory_planner.hpp"
#include "module/fire_control/planner/reference_trajectory_builder.hpp"
#include "module/fire_control/shoot_evaluator.hpp"
#include "module/fire_control/solver/aim_point_sampling.hpp"
#include "module/fire_control/solver/target_solution_solver.hpp"
#include "module/predictor/snapshot.hpp"
#include "utility/math/angle.hpp"
#include "utility/serializable.hpp"

#include <chrono>
#include <cmath>
#include <format>
#include <optional>

using namespace rmcs::kernel;
using namespace rmcs::fire_control;

struct FireControl::Impl {
    struct Config : util::Serializable {
        double initial_bullet_speed; // m/s
        double shoot_delay;          // s
        double yaw_offset;           // rad (config in degree)
        double pitch_offset;         // rad (config in degree)

        // clang-format off
        constexpr static std::tuple metas {
            &Config::initial_bullet_speed, "initial_bullet_speed",
            &Config::shoot_delay,"shoot_delay",
            &Config::yaw_offset,"yaw_offset",
            &Config::pitch_offset,"pitch_offset",
        };
        // clang-format on
    };

    struct AimPointConfig : util::Serializable {
        double coming_angle;
        double leaving_angle;
        double outpost_coming_angle;
        double outpost_leaving_angle;

        constexpr static std::tuple metas {
            &AimPointConfig::coming_angle,
            "coming_angle",
            &AimPointConfig::leaving_angle,
            "leaving_angle",
            &AimPointConfig::outpost_coming_angle,
            "outpost_coming_angle",
            &AimPointConfig::outpost_leaving_angle,
            "outpost_leaving_angle",
        };
    };

    Config config;

    MpcTrajectoryPlanner mpc_trajectory_planner;
    ReferenceTrajectoryBuilder reference_trajectory_builder;
    TargetSolutionSolver target_solution_solver;
    ShootEvaluator shoot_evaluator;
    AimPointChooser aim_point_chooser;

    const double kMinValidBulletSpeed { 10. };

    auto initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
        auto result = config.serialize(yaml);
        if (!result.has_value()) {
            return std::unexpected { result.error() };
        }
        if (!(config.initial_bullet_speed > kMinValidBulletSpeed)) {
            return std::unexpected { std::format(
                "Invalid initial_bullet_speed: {}", config.initial_bullet_speed) };
        }

        config.yaw_offset   = util::deg2rad(config.yaw_offset);
        config.pitch_offset = util::deg2rad(config.pitch_offset);

        auto chooser_source = AimPointConfig {};
        result              = chooser_source.serialize(yaml["aim_point_provider"]);
        if (!result.has_value()) {
            return std::unexpected {
                std::format("Invalid fire_control.aim_point_provider: {}", result.error()),
            };
        }
        aim_point_chooser.initialize(AimPointChooser::Config {
            .coming_angle          = util::deg2rad(chooser_source.coming_angle),
            .leaving_angle         = util::deg2rad(chooser_source.leaving_angle),
            .outpost_coming_angle  = util::deg2rad(chooser_source.outpost_coming_angle),
            .outpost_leaving_angle = util::deg2rad(chooser_source.outpost_leaving_angle),
        });

        auto evaluate_result = shoot_evaluator.configure_yaml(yaml["shoot_evaluator"]);
        if (!evaluate_result.has_value()) {
            return std::unexpected { std::format(
                "shoot_evaluator init failed: {}", evaluate_result.error()) };
        }

        auto planner_result = mpc_trajectory_planner.configure_yaml(yaml["mpc"]);
        if (!planner_result.has_value()) {
            return std::unexpected { std::format(
                "mpc_trajectory_planner init failed: {}", planner_result.error()) };
        }

        return {};
    }

    struct AimCommand {
        double pitch { 0.0 };
        double yaw { 0.0 };
    };

    auto make_result(AimCommand const& command, Eigen::Vector3d const& center_position,
        bool control, double current_yaw) -> Result {
        auto const center_distance = std::hypot(center_position.x(), center_position.y());
        auto shoot_command         = ShootEvaluator::Command {
                    .control          = control,
                    .auto_aim_enabled = control,
                    .yaw              = command.yaw,
                    .distance         = center_distance,
        };
        auto shoot_permitted = shoot_evaluator.evaluate(shoot_command, current_yaw);

        return Result {
            .pitch           = command.pitch,
            .yaw             = command.yaw,
            .shoot_permitted = shoot_permitted,
        };
    }

    auto make_mpc_command(MpcTrajectoryPlanner::Plan const& plan) const -> AimCommand {
        return AimCommand {
            .pitch = plan.pitch,
            .yaw   = plan.yaw,
        };
    }

    auto apply_offset(AimCommand command) const -> AimCommand {
        command.yaw   = util::normalize_angle(command.yaw + config.yaw_offset);
        command.pitch = command.pitch + config.pitch_offset;
        return command;
    }

    auto solve_aim_command(const predictor::Snapshot& snapshot,
        TargetSolutionSolver::TargetSolution const& target_solution)
        -> std::expected<AimCommand, std::string> {
        auto raw_ballistic_direction = AimCommand {
            .pitch = target_solution.impact_pitch,
            .yaw   = target_solution.impact_yaw,
        };

        auto sample_attitude = [&](Clock::time_point t) -> std::expected<AimAttitude, std::string> {
            auto aim_point = sample_aim_point_at(snapshot, aim_point_chooser, t);
            if (!aim_point.has_value()) {
                return std::unexpected { "aim point sample failed" };
            }
            return solve_aim_attitude(*aim_point, config.initial_bullet_speed);
        };

        auto reference =
            reference_trajectory_builder.build(target_solution.impact_time, sample_attitude);
        if (!reference.has_value()) {
            return apply_offset(raw_ballistic_direction);
        }

        auto planned = mpc_trajectory_planner.plan(*reference);
        if (planned.has_value()) {
            return apply_offset(make_mpc_command(*planned));
        }

        auto fallback = AimCommand {
            .pitch = (*reference)(2, kMpcAxisHorizon / 2),
            .yaw   = util::normalize_angle((*reference)(0, kMpcAxisHorizon / 2)),
        };
        return apply_offset(fallback);
    }

    auto solve(const predictor::Snapshot& snapshot, bool control, double current_yaw)
        -> std::optional<Result> {
        auto target_solution = target_solution_solver.solve(
            snapshot, aim_point_chooser, config.initial_bullet_speed, config.shoot_delay);
        if (!target_solution.has_value()) return std::nullopt;

        auto command = solve_aim_command(snapshot, *target_solution);
        if (!command.has_value()) return std::nullopt;

        return make_result(*command, target_solution->center_position, control, current_yaw);
    }
};

FireControl::FireControl() noexcept
    : pimpl { std::make_unique<Impl>() } { };
FireControl::~FireControl() noexcept = default;

auto FireControl::initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
    return pimpl->initialize(yaml);
}

auto FireControl::solve(const predictor::Snapshot& snapshot, bool control, double current_yaw)
    -> std::optional<Result> {
    return pimpl->solve(snapshot, control, current_yaw);
}
