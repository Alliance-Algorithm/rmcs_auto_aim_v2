#include "fire_control.hpp"

#include <chrono>
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

        auto chooser_result = aim_point_chooser.configure_yaml(yaml["aim_point_chooser"]);
        if (!chooser_result.has_value()) {
            return std::unexpected {
                std::format("aim_point_chooser init failed: {}", chooser_result.error()),
            };
        }

        auto evaluate_result = shoot_evaluator.configure_yaml(yaml["shoot_evaluator"]);
        if (!evaluate_result.has_value()) {
            return std::unexpected {
                std::format("shoot_evaluator init failed: {}", evaluate_result.error()),
            };
        }

        auto planner_result = mpc_trajectory_planner.configure_yaml(yaml["mpc"]);
        if (!planner_result.has_value()) {
            return std::unexpected {
                std::format("mpc_trajectory_planner init failed: {}", planner_result.error()),
            };
        }

        return {};
    }

    auto solve(predictor::Snapshot const& snapshot, bool control, double current_yaw)
        -> std::optional<Result> {
        auto target_solution = target_solution_solver.solve(
            snapshot, aim_point_chooser, config.initial_bullet_speed, config.shoot_delay);
        if (!target_solution.has_value()) return std::nullopt;

        auto center_position    = target_solution->center_position;
        auto aim_point_position = target_solution->aim_point;
        auto pitch              = target_solution->impact_pitch;
        auto yaw                = target_solution->impact_yaw;

        auto pitch_rate        = std::numeric_limits<double>::quiet_NaN();
        auto yaw_rate          = std::numeric_limits<double>::quiet_NaN();
        auto pitch_acc         = std::numeric_limits<double>::quiet_NaN();
        auto yaw_acc           = std::numeric_limits<double>::quiet_NaN();
        auto feedforward_valid = false;

        if (config.mpc_enable && snapshot.device_id() != DeviceId::OUTPOST) {
            auto sample_attitude = [&](TimePoint t) -> std::expected<AimAttitude, std::string> {
                auto sample = AimPointSampler::sample_at(
                    snapshot, aim_point_chooser, t, config.initial_bullet_speed);
                if (!sample.has_value()) return std::unexpected { sample.error() };
                return sample->attitude;
            };

            auto reference =
                reference_trajectory_builder.build(target_solution->impact_time, sample_attitude);
            if (reference.has_value()) {
                auto planned = mpc_trajectory_planner.plan(*reference);
                if (planned.has_value()) {
                    pitch             = planned->pitch;
                    yaw               = planned->yaw;
                    pitch_rate        = planned->pitch_rate;
                    yaw_rate          = planned->yaw_rate;
                    pitch_acc         = planned->pitch_acc;
                    yaw_acc           = planned->yaw_acc;
                    feedforward_valid = true;
                } else {
                    constexpr int kYawRow    = 0;
                    constexpr int kPitchRow  = 2;
                    constexpr int kCenterCol = kMpcAxisHorizon / 2;
                    pitch                    = (*reference)(kPitchRow, kCenterCol);
                    yaw = util::normalize_angle((*reference)(kYawRow, kCenterCol));
                }
            }
        }

        yaw   = util::normalize_angle(yaw + config.yaw_offset);
        pitch = pitch + config.pitch_offset;

        auto shoot_command = ShootEvaluator::Command {
            .control            = control,
            .auto_aim_enabled   = control,
            .yaw                = yaw,
            .center_position    = center_position,
            .aim_point_position = aim_point_position,
        };
        auto shoot_permitted = shoot_evaluator.evaluate(shoot_command, current_yaw);

        return Result {
            .pitch             = pitch,
            .yaw               = yaw,
            .pitch_rate        = pitch_rate,
            .yaw_rate          = yaw_rate,
            .pitch_acc         = pitch_acc,
            .yaw_acc           = yaw_acc,
            .feedforward_valid = feedforward_valid,
            .shoot_permitted   = shoot_permitted,
            .center_position   = center_position,
        };
    }

    MpcTrajectoryPlanner mpc_trajectory_planner {};
    ReferenceTrajectoryBuilder reference_trajectory_builder {};
    TargetSolutionSolver target_solution_solver {};
    ShootEvaluator shoot_evaluator {};
    AimPointChooser aim_point_chooser {};
};

FireControl::FireControl() noexcept
    : pimpl { std::make_unique<Impl>() } { }

FireControl::~FireControl() noexcept = default;

auto FireControl::initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
    return pimpl->initialize(yaml);
}

auto FireControl::solve(const predictor::Snapshot& snapshot, bool control, double current_yaw)
    -> std::optional<Result> {
    return pimpl->solve(snapshot, control, current_yaw);
}
