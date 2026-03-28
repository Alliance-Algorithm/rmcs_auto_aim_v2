#include "fire_control.hpp"

#include <cmath>
#include <format>
#include <optional>

#include "module/fire_control/aim_point_chooser.hpp"
#include "module/fire_control/shoot_evaluator.hpp"
#include "module/fire_control/trajectory_solution.hpp"
#include "module/predictor/snapshot.hpp"
#include "utility/logging/printer.hpp"
#include "utility/math/angle.hpp"
#include "utility/serializable.hpp"

using namespace rmcs::kernel;
using namespace rmcs::fire_control;

struct FireControl::Impl {
    struct Config : util::Serializable {
        double initial_bullet_speed; // m/s
        double shoot_delay;          // s
        double yaw_offset;           // rad (config in degree)
        double pitch_offset;         // rad (config in degree)

        double coming_angle;               // rad
        double leaving_angle;              // rad
        double outpost_coming_angle;       // rad
        double outpost_leaving_angle;      // rad
        double angular_velocity_threshold; // rad/s

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
            &Config::angular_velocity_threshold,"angular_velocity_threshold",
        };
        // clang-format on
    };

    Config config;

    AimPointChooser aim_point_chooser;
    ShootEvaluator shoot_evaluator;

    rmcs::Printer log { "FireControl" };

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

        config.yaw_offset                 = util::deg2rad(config.yaw_offset);
        config.pitch_offset               = util::deg2rad(config.pitch_offset);
        config.coming_angle               = util::deg2rad(config.coming_angle);
        config.leaving_angle              = util::deg2rad(config.leaving_angle);
        config.outpost_coming_angle       = util::deg2rad(config.outpost_coming_angle);
        config.outpost_leaving_angle      = util::deg2rad(config.outpost_leaving_angle);
        config.angular_velocity_threshold = util::deg2rad(config.angular_velocity_threshold);

        auto chooser_config = AimPointChooser::Config {
            .coming_angle               = config.coming_angle,
            .leaving_angle              = config.leaving_angle,
            .angular_velocity_threshold = config.angular_velocity_threshold,
            .outpost_coming_angle       = config.outpost_coming_angle,
            .outpost_leaving_angle      = config.outpost_leaving_angle,
        };
        aim_point_chooser.initialize(chooser_config);

        auto evaluate_result = shoot_evaluator.initialize(yaml);
        if (!evaluate_result.has_value()) {
            return std::unexpected { std::format(
                "shoot_evaluator init failed: {}", evaluate_result.error()) };
        }

        return {};
    }

    const int kMaxIterateCount { 5 };
    const double kMaxFlyTimeThreshold { 0.001 };

    auto solve(const predictor::Snapshot& snapshot, Translation const& odom_to_muzzle_translation,
        bool control, double current_yaw) -> std::optional<Result> {
        // 以整车位置来初步迭代飞行时间
        auto state                    = snapshot.ekf_x();
        auto target_position_in_world = Eigen::Vector3d { state[0], state[2], state[4] };

        const double bullet_speed = config.initial_bullet_speed;
        auto current_fly_time     = target_position_in_world.norm() / bullet_speed;

        auto best_armor_opt     = std::optional<Armor3D> {};
        auto trajectory_result  = TrajectorySolution::Output {};
        auto horizon_distance   = 0.0;
        auto has_valid_solution = false;
        int choose_armor_fail_count { 0 };
        int invalid_input_fail_count { 0 };
        int trajectory_fail_count { 0 };

        for (int i = 0; i < kMaxIterateCount; ++i) {
            // 计算预测的时间点 = 子弹飞行时间 + 系统响应延迟
            double total_predict_time = current_fly_time + config.shoot_delay;
            auto t_target             = snapshot.time_stamp()
                + std::chrono::duration_cast<Clock::duration>(
                    std::chrono::duration<double>(total_predict_time));

            auto predicted_armors = snapshot.predicted_armors(t_target);
            auto predicted_ekf_x  = snapshot.predict_at(t_target);

            auto chosen_armor_opt =
                aim_point_chooser.choose_armor(predicted_armors, predicted_ekf_x);
            if (!chosen_armor_opt) {
                ++choose_armor_fail_count;
                if (predicted_armors.empty()) {
                    log.warn("solve failed: no predicted armors (iter={}, predict_time={:.4f}s, "
                             "fly_time={:.4f}s, delay={:.4f}s)",
                        i, total_predict_time, current_fly_time, config.shoot_delay);
                } else {
                    auto center_yaw       = std::atan2(predicted_ekf_x[2], predicted_ekf_x[0]);
                    auto angular_velocity = predicted_ekf_x[7];
                    log.warn("solve failed: choose_armor rejected all candidates (iter={}, "
                             "predicted_armors={}, center_yaw={:.3f}rad, "
                             "angular_velocity={:.3f}rad/s)",
                        i, predicted_armors.size(), center_yaw, angular_velocity);
                }
                continue;
            }
            best_armor_opt = chosen_armor_opt;

            auto const& armor_translation = best_armor_opt->translation;

            auto armor_position_in_world = Eigen::Vector3d {};
            armor_translation.copy_to(armor_position_in_world);

            auto _odom_to_muzzle_translation = Eigen::Vector3d {};
            odom_to_muzzle_translation.copy_to(_odom_to_muzzle_translation);

            auto bullet_in_muzzle = armor_position_in_world - _odom_to_muzzle_translation;

            auto target_d = std::sqrt(bullet_in_muzzle.x() * bullet_in_muzzle.x()
                + bullet_in_muzzle.y() * bullet_in_muzzle.y());
            auto target_h = bullet_in_muzzle.z();
            if (!(target_d > 0.0)) {
                ++invalid_input_fail_count;
                log.warn("solve failed: invalid trajectory input (iter={}, target_d={:.6f}, "
                         "target_h={:.6f}, bullet_in_muzzle=[{:.3f}, {:.3f}, {:.3f}])",
                    i, target_d, target_h, bullet_in_muzzle.x(), bullet_in_muzzle.y(),
                    bullet_in_muzzle.z());
                continue;
            }

            auto solution           = TrajectorySolution {};
            solution.input.v0       = bullet_speed;
            solution.input.target_d = target_d;
            solution.input.target_h = target_h;

            auto result = solution.solve();

            if (!result) {
                ++trajectory_fail_count;
                log.warn("solve failed: trajectory solver did not converge (iter={}, v0={:.3f}, "
                         "target_d={:.3f}, target_h={:.3f})",
                    i, solution.input.v0, solution.input.target_d, solution.input.target_h);
                continue;
            }

            has_valid_solution = true;
            auto time_error    = std::abs(result->fly_time - current_fly_time);
            current_fly_time   = result->fly_time;
            trajectory_result  = *result;
            horizon_distance   = target_d;

            if (time_error < kMaxFlyTimeThreshold) break;
        }

        if (!has_valid_solution || !best_armor_opt) {
            log.warn("solve failed: exhausted {} iterations (choose_armor_fail={}, "
                     "invalid_input_fail={}, trajectory_fail={})",
                kMaxIterateCount, choose_armor_fail_count, invalid_input_fail_count,
                trajectory_fail_count);
            return std::nullopt;
        }

        auto best_armor_position_in_world = Eigen::Vector3d {};
        best_armor_opt->translation.copy_to(best_armor_position_in_world);

        auto odom_to_muzzle = Eigen::Vector3d {};
        odom_to_muzzle_translation.copy_to(odom_to_muzzle);

        // Keep yaw/pitch in the same reference: both are solved from target position relative to
        // muzzle in odom frame.
        auto target_in_muzzle = best_armor_position_in_world - odom_to_muzzle;
        auto final_yaw =
            std::atan2(best_armor_position_in_world.y(), best_armor_position_in_world.x());
        final_yaw += config.yaw_offset;

        auto command = ShootEvaluator::Command {
            .control          = control,
            .auto_aim_enabled = control,
            .aim_point_valid  = true,
            .yaw              = final_yaw,
            .distance         = horizon_distance,
        };
        auto shoot_permitted = shoot_evaluator.evaluate(command, current_yaw);

        return Result {
            .pitch            = trajectory_result.pitch + config.pitch_offset,
            .yaw              = final_yaw,
            .horizon_distance = horizon_distance,
            .shoot_permitted  = shoot_permitted,
        };
    }
};

FireControl::FireControl() noexcept
    : pimpl { std::make_unique<Impl>() } { };
FireControl::~FireControl() noexcept = default;

auto FireControl::initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
    return pimpl->initialize(yaml);
}

auto FireControl::solve(const predictor::Snapshot& snapshot,
    Translation const& odom_to_muzzle_translation, bool control, double current_yaw)
    -> std::optional<Result> {
    return pimpl->solve(snapshot, odom_to_muzzle_translation, control, current_yaw);
}
