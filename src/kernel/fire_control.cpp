#include "fire_control.hpp"

#include <cmath>
#include <format>
#include <limits>
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
        bool mpc_enable;
        double yaw_offset;   // rad (config in degree)
        double pitch_offset; // rad (config in degree)

        // clang-format off
        constexpr static std::tuple metas {
            &Config::initial_bullet_speed, "initial_bullet_speed",
            &Config::shoot_delay,"shoot_delay",
            &Config::mpc_enable,"mpc_enable",
            &Config::yaw_offset,"yaw_offset",
            &Config::pitch_offset,"pitch_offset",
        };
        // clang-format on
    } config;

    MpcTrajectoryPlanner mpc_trajectory_planner;
    ReferenceTrajectoryBuilder reference_trajectory_builder;
    TargetSolutionSolver target_solution_solver;
    ShootEvaluator shoot_evaluator;
    AimPointChooser aim_point_chooser;

    auto initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
        auto result = config.serialize(yaml);
        if (!result.has_value()) return std::unexpected { result.error() };

        if (!(config.initial_bullet_speed > 0.)) {
            return std::unexpected { std::format("Initial_bullet_speed must > 0 m/s") };
        }

        config.yaw_offset   = util::deg2rad(config.yaw_offset);
        config.pitch_offset = util::deg2rad(config.pitch_offset);

        auto chooser_result = aim_point_chooser.configure_yaml(yaml["aim_point_chooser"]);
        if (!chooser_result.has_value()) {
            return std::unexpected {
                std::format("aim_point_chooser init failed: {}", chooser_result.error()),
            };
        }

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
        double pitch_rate { std::numeric_limits<double>::quiet_NaN() };
        double yaw_rate { std::numeric_limits<double>::quiet_NaN() };
        double pitch_acc { std::numeric_limits<double>::quiet_NaN() };
        double yaw_acc { std::numeric_limits<double>::quiet_NaN() };
        bool feedforward_valid { false };
    };

    static auto make_shoot_command(AimCommand const& command,
        Eigen::Vector3d const& center_position, bool control) -> ShootEvaluator::Command {
        auto const center_distance = std::hypot(center_position.x(), center_position.y());

        return ShootEvaluator::Command {
            .control          = control,
            .auto_aim_enabled = control,
            .yaw              = command.yaw,
            .distance         = center_distance,
        };
    }

    auto make_result(AimCommand const& command, Eigen::Vector3d const& center_position,
        bool control, double current_yaw) -> Result {
        auto shoot_command   = make_shoot_command(command, center_position, control);
        auto shoot_permitted = shoot_evaluator.evaluate(shoot_command, current_yaw);

        return Result {
            .pitch             = command.pitch,
            .yaw               = command.yaw,
            .pitch_rate        = command.pitch_rate,
            .yaw_rate          = command.yaw_rate,
            .pitch_acc         = command.pitch_acc,
            .yaw_acc           = command.yaw_acc,
            .feedforward_valid = command.feedforward_valid,
            .shoot_permitted   = shoot_permitted,
        };
    }

    auto apply_offset(AimCommand command) const -> AimCommand {
        command.yaw   = util::normalize_angle(command.yaw + config.yaw_offset);
        command.pitch = command.pitch + config.pitch_offset;
        return command;
    }

    auto resolve_aim_command(const predictor::Snapshot& snapshot,
        TargetSolutionSolver::TargetSolution const& target_solution)
        -> std::expected<AimCommand, std::string> {
        auto const ballistic_command = AimCommand {
            .pitch = target_solution.impact_pitch,
            .yaw   = target_solution.impact_yaw,
        };
        auto raw_command = ballistic_command;

        if (config.mpc_enable) {
            auto sample_attitude =
                [&](Clock::time_point t) -> std::expected<AimAttitude, std::string> {
                return AimPointSampler::sample_attitude_at(
                    snapshot, aim_point_chooser, t, config.initial_bullet_speed);
            };

            auto reference =
                reference_trajectory_builder.build(target_solution.impact_time, sample_attitude);
            if (reference.has_value()) {
                auto planned = mpc_trajectory_planner.plan(*reference);
                if (planned.has_value()) {
                    raw_command = AimCommand {
                        .pitch             = planned->pitch,
                        .yaw               = planned->yaw,
                        .pitch_rate        = planned->pitch_rate,
                        .yaw_rate          = planned->yaw_rate,
                        .pitch_acc         = planned->pitch_acc,
                        .yaw_acc           = planned->yaw_acc,
                        .feedforward_valid = true,
                    };
                } else {
                    constexpr int kYawRow   = 0;
                    constexpr int kPitchRow = 2;
                    auto const kCenterCol   = kMpcAxisHorizon / 2;

                    raw_command = AimCommand {
                        .pitch = (*reference)(kPitchRow, kCenterCol),
                        .yaw   = util::normalize_angle((*reference)(kYawRow, kCenterCol)),
                    };
                }
            }
        }

        return apply_offset(raw_command);
    }

    auto solve(const predictor::Snapshot& snapshot, bool control, double current_yaw)
        -> std::optional<Result> {
        auto target_solution = target_solution_solver.solve(
            snapshot, aim_point_chooser, config.initial_bullet_speed, config.shoot_delay);
        if (!target_solution.has_value()) return std::nullopt;

        auto command = resolve_aim_command(snapshot, *target_solution);
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
