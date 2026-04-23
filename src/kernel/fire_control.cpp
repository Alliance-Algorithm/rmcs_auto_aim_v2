#include "fire_control.hpp"
#include "module/fire_control/aim_point_chooser.hpp"
#include "module/fire_control/planner/mpc_trajectory_planner.hpp"
#include "module/fire_control/shoot_evaluator.hpp"
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

        double coming_angle;          // rad
        double leaving_angle;         // rad
        double outpost_coming_angle;  // rad
        double outpost_leaving_angle; // rad

        // clang-format off
        constexpr static std::tuple metas {
            &Config::initial_bullet_speed, "initial_bullet_speed",
            &Config::shoot_delay,"shoot_delay",
            &Config::yaw_offset,"yaw_offset",
            &Config::pitch_offset,"pitch_offset",

            &Config::coming_angle,"coming_angle",
            &Config::leaving_angle,"leaving_angle",
            &Config::outpost_coming_angle,"outpost_coming_angle",
            &Config::outpost_leaving_angle,"outpost_leaving_angle",
        };
        // clang-format on
    };

    Config config;

    AimPointChooser aim_point_chooser;
    MpcTrajectoryPlanner mpc_trajectory_planner;
    ShootEvaluator shoot_evaluator;

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

        config.yaw_offset            = util::deg2rad(config.yaw_offset);
        config.pitch_offset          = util::deg2rad(config.pitch_offset);
        config.coming_angle          = util::deg2rad(config.coming_angle);
        config.leaving_angle         = util::deg2rad(config.leaving_angle);
        config.outpost_coming_angle  = util::deg2rad(config.outpost_coming_angle);
        config.outpost_leaving_angle = util::deg2rad(config.outpost_leaving_angle);

        auto chooser_config = AimPointChooser::Config {
            .coming_angle          = config.coming_angle,
            .leaving_angle         = config.leaving_angle,
            .outpost_coming_angle  = config.outpost_coming_angle,
            .outpost_leaving_angle = config.outpost_leaving_angle,
        };
        aim_point_chooser.initialize(chooser_config);

        auto evaluate_result = shoot_evaluator.initialize(yaml);
        if (!evaluate_result.has_value()) {
            return std::unexpected { std::format(
                "shoot_evaluator init failed: {}", evaluate_result.error()) };
        }

        auto planner_result = mpc_trajectory_planner.initialize(yaml, chooser_config);
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
        Armor3D armor;
        Clock::time_point impact_time;
    };

    auto make_command(const Armor3D& armor) const -> std::optional<AimCommand> {
        auto armor_position_in_world = Eigen::Vector3d {};
        armor.translation.copy_to(armor_position_in_world);

        auto target_d = std::sqrt(armor_position_in_world.x() * armor_position_in_world.x()
            + armor_position_in_world.y() * armor_position_in_world.y());
        auto target_h = armor_position_in_world.z();
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

        auto final_yaw = std::atan2(armor_position_in_world.y(), armor_position_in_world.x());
        final_yaw += config.yaw_offset;

        return AimCommand {
            .pitch            = trajectory_result->pitch + config.pitch_offset,
            .yaw              = final_yaw,
            .horizon_distance = target_d,
        };
    }

    auto make_result(AimCommand const& command, bool control, double current_yaw) -> Result {
        auto shoot_command = ShootEvaluator::Command {
            .control          = control,
            .auto_aim_enabled = control,
            .aim_point_valid  = true,
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

    auto solve_target(const predictor::Snapshot& snapshot) -> std::optional<ImpactTarget> {
        auto target_kinematics = snapshot.kinematics();

        // 以整车位置来初步迭代飞行时间
        auto target_position_in_world = target_kinematics.center_position;

        const double bullet_speed = config.initial_bullet_speed;
        auto current_fly_time     = target_position_in_world.norm() / bullet_speed;

        auto best_armor_opt = std::optional<Armor3D> { };

        for (int i = 0; i < kMaxIterateCount; ++i) {
            // 计算预测的时间点 = 子弹飞行时间 + 系统响应延迟
            double total_predict_time = current_fly_time + config.shoot_delay;
            auto t_target             = snapshot.time_stamp()
                + std::chrono::duration_cast<Clock::duration>(
                    std::chrono::duration<double>(total_predict_time));

            auto predicted_armors     = snapshot.predicted_armors(t_target);
            auto predicted_kinematics = snapshot.kinematics_at(t_target);

            auto chosen_armor_opt = aim_point_chooser.choose_armor(predicted_armors,
                predicted_kinematics.center_position, predicted_kinematics.angular_velocity);
            if (!chosen_armor_opt) {
                continue;
            }
            best_armor_opt = chosen_armor_opt;

            auto armor_position_in_world = Eigen::Vector3d { };
            best_armor_opt->translation.copy_to(armor_position_in_world);

            auto target_d = std::sqrt(armor_position_in_world.x() * armor_position_in_world.x()
                + armor_position_in_world.y() * armor_position_in_world.y());
            if (!(target_d > 0.0)) {
                continue;
            }

            auto solution           = TrajectorySolution { };
            solution.input.v0       = bullet_speed;
            solution.input.target_d = target_d;
            solution.input.target_h = armor_position_in_world.z();

            auto result = solution.solve();
            if (!result) {
                continue;
            }

            auto time_error  = std::abs(result->fly_time - current_fly_time);
            current_fly_time = result->fly_time;
            if (time_error < kMaxFlyTimeThreshold) break;
        }

        if (!best_armor_opt) return std::nullopt;

        auto const impact_time = snapshot.time_stamp()
            + std::chrono::duration_cast<Clock::duration>(
                std::chrono::duration<double>(current_fly_time + config.shoot_delay));

        return ImpactTarget {
            .armor       = *best_armor_opt,
            .impact_time = impact_time,
        };
    }

    auto solve(const predictor::Snapshot& snapshot, bool control, double current_yaw)
        -> std::optional<Result> {
        auto impact_target = solve_target(snapshot);
        if (!impact_target) return std::nullopt;

        auto direct_command = make_command(impact_target->armor);
        if (!direct_command) return std::nullopt;

        auto final_command = *direct_command;
        if (auto planned = mpc_trajectory_planner.plan(snapshot, impact_target->impact_time,
                config.initial_bullet_speed, config.yaw_offset, config.pitch_offset)) {
            final_command.yaw   = planned->yaw;
            final_command.pitch = planned->pitch;
        }

        return make_result(final_command, control, current_yaw);
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
