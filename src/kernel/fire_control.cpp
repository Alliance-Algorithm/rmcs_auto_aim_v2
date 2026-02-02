#include "fire_control.hpp"

#include "module/fire_control/aim_point_chooser.hpp"
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
        double shoot_offset_x;       // m
        double shoot_offset_y;       // m
        double shoot_offset_z;       // m

        double k;          // 基础阻力系数 (小弹丸~0.019, 大弹丸~0.005)
        double bias_scale; // 动态补偿系数：修正额外阻力,阻力越大，该系数越大，default=1

        double coming_angle;               // rad
        double leaving_angle;              // rad
        double outpost_coming_angle;       // rad
        double outpost_leaving_angle;      // rad
        double angular_velocity_threshold; // rad/s

        // clang-format off
        constexpr static std::tuple metas {
            &Config::initial_bullet_speed, "initial_bullet_speed",
            &Config::shoot_delay,"shoot_delay",
            &Config::shoot_offset_x,"shoot_offset_x",
            &Config::shoot_offset_y,"shoot_offset_y",
            &Config::shoot_offset_z,"shoot_offset_z",

            &Config::k,"k",
            &Config::bias_scale,"bias_scale",

            &Config::coming_angle,"coming_angle",
            &Config::leaving_angle,"leaving_angle",
            &Config::outpost_coming_angle,"outpost_coming_angle",
            &Config::outpost_leaving_angle,"outpost_leaving_angle",
            &Config::angular_velocity_threshold,"angular_velocity_threshold",
        };
        // clang-format on
    };

    Config config;

    double bullet_speed_buffer { 0. };
    double bullet_speed { 0. };

    AimPointChooser aim_point_chooser;

    rmcs::Printer log { "FireControl" };

    auto initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
        auto result = config.serialize(yaml);
        if (!result.has_value()) {
            return std::unexpected { result.error() };
        }

        bullet_speed = config.initial_bullet_speed;

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

        return {};
    }

    const int kMaxIterateCount { 5 };
    const double kMaxFlyTimeThreshold { 0.001 };

    auto set_bullet_speed(double speed) -> void { bullet_speed_buffer = speed; }

    auto solve(const predictor::Snapshot& snapshot, Translation const& odom_to_muzzle_translation)
        -> std::optional<Result> {
        auto state                    = snapshot.ekf_x();
        auto target_position_in_world = Eigen::Vector3d { state[0], state[2], state[4] };

        if (bullet_speed_buffer > 10.) {
            bullet_speed = bullet_speed_buffer;
        } else {
            bullet_speed = config.initial_bullet_speed;
        }

        auto current_fly_time = target_position_in_world.norm() / bullet_speed;

        auto best_armor_opt    = std::optional<Armor3D> {};
        auto trajectory_result = TrajectorySolution::Output {};
        auto horizon_distance  = 0.0;

        auto solution_params       = fire_control::TrajectorySolution::TrajectoryParams {};
        solution_params.k          = config.k;
        solution_params.bias_scale = config.bias_scale;

        for (int i = 0; i < kMaxIterateCount; ++i) {
            // 计算预测的时间点 = 子弹飞行时间 + 系统响应延迟
            double total_predict_time = current_fly_time + config.shoot_delay;
            auto t_target             = snapshot.time_stamp()
                + std::chrono::duration_cast<Clock::duration>(
                    std::chrono::duration<double>(total_predict_time));

            auto predicted_armors = snapshot.predicted_armors(t_target);
            auto predicted_ekf_x  = snapshot.predict_at(t_target);

            best_armor_opt = aim_point_chooser.choose_armor(predicted_armors, predicted_ekf_x);
            if (!best_armor_opt) return std::nullopt;

            auto const& armor_translation = best_armor_opt->translation;

            auto armor_position_in_world = Eigen::Vector3d {};
            armor_translation.copy_to(armor_position_in_world);

            auto _odom_to_muzzle_translation = Eigen::Vector3d {};
            odom_to_muzzle_translation.copy_to(_odom_to_muzzle_translation);

            auto bullet_in_muzzle = armor_position_in_world - _odom_to_muzzle_translation;

            auto target_d = std::sqrt(bullet_in_muzzle.x() * bullet_in_muzzle.x()
                + bullet_in_muzzle.y() * bullet_in_muzzle.y());
            auto target_h = bullet_in_muzzle.z();

            auto solution           = TrajectorySolution {};
            solution.input.v0       = bullet_speed;
            solution.input.target_d = target_d;
            solution.input.target_h = target_h;
            solution.input.params   = solution_params;

            auto result = solution.solve();

            if (!result) {
                return std::nullopt;
            }

            auto time_error   = std::abs(result->fly_time - current_fly_time);
            current_fly_time  = result->fly_time;
            trajectory_result = *result;
            horizon_distance  = target_d;

            if (time_error < kMaxFlyTimeThreshold) break;
        }

        auto final_yaw = std::atan2(best_armor_opt->translation.y, best_armor_opt->translation.x);

        return Result {
            .pitch            = trajectory_result.pitch,
            .yaw              = final_yaw,
            .horizon_distance = horizon_distance,
        };
    }
};

FireControl::FireControl() noexcept
    : pimpl { std::make_unique<Impl>() } { };
FireControl::~FireControl() noexcept = default;

auto FireControl::initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
    return pimpl->initialize(yaml);
}

auto FireControl::set_bullet_speed(double speed) -> void { return pimpl->set_bullet_speed(speed); }

auto FireControl::solve(const predictor::Snapshot& snapshot,
    Translation const& odom_to_muzzle_translation) -> std::optional<Result> {
    return pimpl->solve(snapshot, odom_to_muzzle_translation);
}
