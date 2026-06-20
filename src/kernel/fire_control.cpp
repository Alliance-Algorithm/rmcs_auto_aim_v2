#include "fire_control.hpp"

#include <chrono>
#include <cmath>
#include <format>
#include <memory>
#include <optional>

#include "module/fire_control/armor_selector.hpp"
#include "module/fire_control/planner/trajectory_planner.hpp"
#include "module/fire_control/shoot_evaluator.hpp"
#include "module/fire_control/target_solver.hpp"
#include "module/predictor/snapshot.hpp"
#include "utility/math/angle.hpp"
#include "utility/serializable.hpp"

using namespace rmcs::kernel;
using namespace rmcs::fire_control;

struct FireControl::Impl {
    struct Config : util::Serializable {
        double initial_bullet_speed;
        double shoot_delay;
        bool mpc_enable;
        double yaw_offset;
        double pitch_offset;

        constexpr static std::tuple metas {
            &Config::initial_bullet_speed,
            "initial_bullet_speed",
            &Config::shoot_delay,
            "shoot_delay",
            &Config::mpc_enable,
            "mpc_enable",
            &Config::yaw_offset,
            "yaw_offset",
            &Config::pitch_offset,
            "pitch_offset",
        };
    } config;

    auto initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
        auto result = config.serialize(yaml);
        if (!result.has_value()) return std::unexpected { result.error() };

        if (!(config.initial_bullet_speed > 0.0)) {
            return std::unexpected { std::format("Initial_bullet_speed must > 0 m/s") };
        }

        config.yaw_offset   = util::deg2rad(config.yaw_offset);
        config.pitch_offset = util::deg2rad(config.pitch_offset);

        auto selector_result = armor_selector.configure_yaml(yaml["armor_target_selector"]);
        if (!selector_result.has_value()) {
            return std::unexpected {
                std::format("armor_target_selector init failed: {}", selector_result.error()),
            };
        }

        auto evaluate_result = shoot_evaluator.configure_yaml(yaml["shoot_evaluator"]);
        if (!evaluate_result.has_value()) {
            return std::unexpected {
                std::format("shoot_evaluator init failed: {}", evaluate_result.error()),
            };
        }

        auto planner_result = trajectory_planner.configure_yaml(yaml["mpc"]);
        if (!planner_result.has_value()) {
            return std::unexpected {
                std::format("trajectory_planner init failed: {}", planner_result.error()),
            };
        }

        return { };
    }

    auto solve(predictor::Snapshot const& snapshot, GimbalState const& gimbal_state)
        -> std::optional<Result> {

        /// 1. 目标选择
        auto selected_armor_id  = int {};
        auto aim_point_position = Eigen::Vector3d {};
        auto center_position    = Eigen::Vector3d {};
        {
            auto const target_motion = snapshot.motion();
            auto const coarse_fly_time =
                target_motion.center_position.norm() / config.initial_bullet_speed;
            auto const selection_predict_time = config.shoot_delay + coarse_fly_time;
            auto const selection_time         = snapshot.time_stamp()
                + std::chrono::duration_cast<Duration>(
                    std::chrono::duration<double>(selection_predict_time));

            auto target_candidates = target_solver.make_candidates(snapshot, selection_time);
            auto selected_index = armor_selector.select(target_candidates, last_selected_armor_id);

            if (!selected_index.has_value()) {
                last_selected_armor_id.reset();
                return std::nullopt;
            }

            auto const& selected = target_candidates[*selected_index];
            selected_armor_id    = selected.armor.id;
            center_position      = target_motion.center_position;
            selected.armor.translation.copy_to(aim_point_position);
        }

        /// 2. 弹道解算
        auto target_solution = TargetSolution {};
        {
            auto solution = target_solver.solve(snapshot, selected_armor_id, gimbal_state.timestamp,
                config.initial_bullet_speed, config.shoot_delay);

            if (!solution.has_value()) {
                last_selected_armor_id.reset();
                return std::nullopt;
            }

            last_selected_armor_id = solution->candidate.armor.id;
            target_solution        = std::move(*solution);
        }

        /// 3. 轨迹规划
        auto feedforward = std::optional<Feedforward> {};
        if (config.mpc_enable) {
            auto planned = trajectory_planner.plan(snapshot, target_solution,
                gimbal_state.timestamp, config.initial_bullet_speed, config.shoot_delay);

            if (planned.has_value()) {
                feedforward = Feedforward {
                    .yaw        = planned->yaw,
                    .pitch      = planned->pitch,
                    .pitch_rate = planned->pitch_rate,
                    .yaw_rate   = planned->yaw_rate,
                    .pitch_acc  = planned->pitch_acc,
                    .yaw_acc    = planned->yaw_acc,
                };
            }
        }

        /// 4. 偏移校正
        auto yaw   = feedforward.has_value() ? feedforward->yaw : target_solution.attitude.yaw;
        auto pitch = feedforward.has_value() ? feedforward->pitch : target_solution.attitude.pitch;
        yaw        = util::normalize_angle(yaw + config.yaw_offset);
        pitch      = pitch + config.pitch_offset;

        /// 5. 射击评估
        auto shoot_permitted = bool {};
        {
            auto shoot_command = ShootEvaluator::Command {
                .yaw                = yaw,
                .pitch              = pitch,
                .center_position    = center_position,
                .aim_point_position = aim_point_position,
            };
            shoot_permitted = shoot_evaluator.evaluate(shoot_command, gimbal_state);
        }

        // 6. 组装结果
        return Result {
            .pitch           = pitch,
            .yaw             = yaw,
            .feedforward     = feedforward,
            .shoot_permitted = shoot_permitted,
            .center_position = Point3d { center_position },
            .aim_point       = Point3d { aim_point_position },
            .impact_time     = target_solution.impact_time,
        };
    }

    TrajectoryPlanner trajectory_planner { };
    TargetSolver target_solver { };
    ShootEvaluator shoot_evaluator { };
    ArmorSelector armor_selector { };
    std::optional<int> last_selected_armor_id { };
};

FireControl::FireControl() noexcept
    : pimpl { std::make_unique<Impl>() } { }

FireControl::~FireControl() noexcept = default;

auto FireControl::initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
    return pimpl->initialize(yaml);
}

auto FireControl::solve(const predictor::Snapshot& snapshot, GimbalState const& gimbal_state)
    -> std::optional<Result> {
    return pimpl->solve(snapshot, gimbal_state);
}
