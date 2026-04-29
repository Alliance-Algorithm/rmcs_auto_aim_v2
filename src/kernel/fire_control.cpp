#include "fire_control.hpp"

#include <chrono>
#include <cmath>
#include <format>
#include <memory>
#include <optional>

#include "module/fire_control/planner/mpc_trajectory_planner.hpp"
#include "module/fire_control/shoot_evaluator.hpp"
#include "module/fire_control/strategy/aim_point_provider.hpp"
#include "module/fire_control/strategy/fire_control_adapter.hpp"
#include "module/fire_control/strategy/fire_gate_evaluator.hpp"
#include "module/fire_control/trajectory_solution.hpp"
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
        bool is_lazy_gimbal;

        // clang-format off
        constexpr static std::tuple metas {
            &Config::initial_bullet_speed, "initial_bullet_speed",
            &Config::shoot_delay,"shoot_delay",
            &Config::yaw_offset,"yaw_offset",
            &Config::pitch_offset,"pitch_offset",
            &Config::is_lazy_gimbal,"is_lazy_gimbal",
        };
        // clang-format on
    };

    Config config;

    MpcTrajectoryPlanner mpc_trajectory_planner;
    ShootEvaluator shoot_evaluator;
    AimPointProvider aim_point_provider;
    FireControlAdapter fire_control_adapter;
    FireGateEvaluator fire_gate_evaluator;

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
        fire_control_adapter.initialize(config.is_lazy_gimbal);

        auto provider_result = aim_point_provider.configure_yaml(yaml["aim_point_provider"]);
        if (!provider_result.has_value()) {
            return std::unexpected {
                std::format("Invalid fire_control.aim_point_provider: {}", provider_result.error()),
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

    const int kMaxIterateCount { 5 };
    const double kMaxFlyTimeThreshold { 0.001 };

    struct AimCommand {
        double pitch;
        double yaw;
        double horizon_distance;
    };

    struct ImpactTarget {
        TimePoint sample_time;
        Eigen::Vector3d aim_position;
        Clock::time_point impact_time;
    };

    auto make_command(Eigen::Vector3d const& aim_position) const -> std::optional<AimCommand> {
        auto target_d =
            std::sqrt(aim_position.x() * aim_position.x() + aim_position.y() * aim_position.y());
        auto target_h = aim_position.z();
        if (!(target_d > 0.0)) {
            return std::nullopt;
        }

        auto solution           = TrajectorySolution { };
        solution.input.v0       = config.initial_bullet_speed;
        solution.input.target_d = target_d;
        solution.input.target_h = target_h;

        auto trajectory_result = solution.solve();
        if (!trajectory_result) {
            return std::nullopt;
        }

        auto final_yaw = std::atan2(aim_position.y(), aim_position.x());
        final_yaw += config.yaw_offset;

        return AimCommand {
            .pitch            = trajectory_result->pitch + config.pitch_offset,
            .yaw              = final_yaw,
            .horizon_distance = target_d,
        };
    }

    auto make_result(const predictor::Snapshot& snapshot, AimCommand const& command, bool control,
        double current_yaw, TimePoint sample_time, Eigen::Vector3d const& aim_position,
        GateMode gate_mode) -> Result {
        auto fire_context = FireGateContext {
            .snapshot     = snapshot,
            .sample_time  = sample_time,
            .aim_position = aim_position,
            .control      = control,
        };
        auto fire_gate = fire_gate_evaluator.evaluate(gate_mode, fire_context);

        auto shoot_command = ShootEvaluator::Command {
            .control          = control,
            .auto_aim_enabled = control,
            .aim_point_valid  = fire_gate.allowed,
            .yaw              = command.yaw,
            .distance         = command.horizon_distance,
        };
        auto shoot_permitted = shoot_evaluator.evaluate(shoot_command, current_yaw);

        return Result {
            .pitch            = command.pitch,
            .yaw              = command.yaw,
            .horizon_distance = command.horizon_distance,
            .shoot_permitted  = shoot_permitted,
        };
    }

    auto solve_target(const predictor::Snapshot& snapshot, AimPointProvider::Mode mode)
        -> std::optional<ImpactTarget> {
        auto target_kinematics = snapshot.kinematics();

        // 以整车位置来初步迭代飞行时间
        auto target_position_in_world = target_kinematics.center_position;

        const double bullet_speed = config.initial_bullet_speed;
        auto current_fly_time     = target_position_in_world.norm() / bullet_speed;

        auto best_target_opt  = std::optional<Eigen::Vector3d> {};
        auto best_sample_time = snapshot.time_stamp();

        for (int i = 0; i < kMaxIterateCount; ++i) {
            // 计算预测的时间点 = 子弹飞行时间 + 系统响应延迟
            double total_predict_time = current_fly_time + config.shoot_delay;
            auto t_target             = snapshot.time_stamp()
                + std::chrono::duration_cast<Clock::duration>(
                    std::chrono::duration<double>(total_predict_time));

            auto target_opt = aim_point_provider.aim_point_at(snapshot, t_target, mode);
            if (!target_opt) {
                continue;
            }
            best_target_opt  = target_opt;
            best_sample_time = t_target;

            auto const& aim_position = *best_target_opt;

            auto target_d = std::sqrt(
                aim_position.x() * aim_position.x() + aim_position.y() * aim_position.y());
            if (!(target_d > 0.0)) {
                continue;
            }

            auto solution           = TrajectorySolution { };
            solution.input.v0       = bullet_speed;
            solution.input.target_d = target_d;
            solution.input.target_h = aim_position.z();

            auto result = solution.solve();
            if (!result) {
                continue;
            }

            auto time_error  = std::abs(result->fly_time - current_fly_time);
            current_fly_time = result->fly_time;
            if (time_error < kMaxFlyTimeThreshold) break;
        }

        if (!best_target_opt) return std::nullopt;

        auto const impact_time = snapshot.time_stamp()
            + std::chrono::duration_cast<Clock::duration>(
                std::chrono::duration<double>(current_fly_time + config.shoot_delay));

        return ImpactTarget {
            .sample_time  = best_sample_time,
            .aim_position = *best_target_opt,
            .impact_time  = impact_time,
        };
    }

    auto solve(const predictor::Snapshot& snapshot, bool control, double current_yaw)
        -> std::optional<Result> {
        auto const policy  = fire_control_adapter.resolve(snapshot);
        auto impact_target = solve_target(snapshot, policy.aim_mode);
        if (!impact_target) return std::nullopt;

        auto direct_command = make_command(impact_target->aim_position);
        if (!direct_command) return std::nullopt;

        auto final_command    = *direct_command;
        auto sample_aim_point = [&](TimePoint t) {
            return aim_point_provider.aim_point_at(snapshot, t, policy.aim_mode);
        };

        if (auto planned =
                mpc_trajectory_planner.plan(impact_target->impact_time, config.initial_bullet_speed,
                    config.yaw_offset, config.pitch_offset, sample_aim_point)) {
            final_command.yaw   = planned->yaw;
            final_command.pitch = planned->pitch;
        }

        return make_result(snapshot, final_command, control, current_yaw,
            impact_target->sample_time, impact_target->aim_position, policy.gate_mode);
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
