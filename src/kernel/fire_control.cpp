#include "fire_control.hpp"

#include "module/fire_control/aim_point_chooser.hpp"
#include "module/fire_control/trajectory_solution.hpp"
#include "module/predictor/snapshot.hpp"
#include "utility/math/angle.hpp"
#include "utility/serializable.hpp"

using namespace rmcs::kernel;
using namespace rmcs::fire_control;
struct FireControl::Impl {
    struct Config : util::Serializable {
        double v_initial;      // m/s
        double shoot_delay;    // s
        double shoot_offset_x; // m
        double shoot_offset_y; // m
        double shoot_offset_z; // m

        double k;          // 基础阻力系数 (小弹丸~0.019, 大弹丸~0.005)
        double g;          // 重力加速度 (通常取 9.78~9.8)
        double bias_scale; // 动态补偿系数：修正额外阻力,阻力越大，该系数越大，default=1

        double coming_angle;               // rad
        double leaving_angle;              // rad
        double outpost_coming_angle;       // rad
        double outpost_leaving_angle;      // rad
        double angular_velocity_threshold; // rad/s

        constexpr static std::tuple metas {
            &Config::v_initial,
            "v_initial",
            &Config::shoot_delay,
            "shoot_delay",
            &Config::shoot_offset_x,
            "shoot_offset_x",
            &Config::shoot_offset_y,
            "shoot_offset_y",
            &Config::shoot_offset_z,
            "shoot_offset_z",

            &Config::k,
            "k",
            &Config::g,
            "g",
            &Config::bias_scale,
            "bias_scale",

            &Config::coming_angle,
            "coming_angle",
            &Config::leaving_angle,
            "leaving_angle",
            &Config::outpost_coming_angle,
            "outpost_coming_angle",
            &Config::outpost_leaving_angle,
            "outpost_leaving_angle",
            &Config::angular_velocity_threshold,
            "angular_velocity_threshold",
        };
    };

    Config config;

    AimPointChooser aim_point_chooser;

    auto initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
        auto result = config.serialize(yaml);
        if (!result.has_value()) {
            return std::unexpected { result.error() };
        }

        // 配置文件中角度是度制、延迟是毫秒，内部统一转成弧度/秒
        config.coming_angle               = util::deg2rad(config.coming_angle);
        config.leaving_angle              = util::deg2rad(config.leaving_angle);
        config.outpost_coming_angle       = util::deg2rad(config.outpost_coming_angle);
        config.outpost_leaving_angle      = util::deg2rad(config.outpost_leaving_angle);
        config.angular_velocity_threshold = util::deg2rad(config.angular_velocity_threshold);
        config.shoot_delay /= 1000.0; // ms -> s

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
    const double kMaxFlyTimeThreold { 0.001 };

    auto solve(const predictor::Snapshot& snapshot) -> std::optional<Result> {
        // 1. 初始猜测：以当前目标中心距离估算初次飞行时间
        auto state = snapshot.ekf_x();
        auto x = state[0], y = state[2], z = state[4];
        auto distance = std::sqrt(x * x + y * y + z * z);

        auto current_fly_time = distance / config.v_initial;

        // 迭代中间变量
        auto iter_pitch        = 0.0;
        auto best_armor_opt    = std::optional<Armor3D> {};
        auto trajectory_result = TrajectorySolution::Output {};
        auto horizon_distance  = 0.0;

        // 2. 迭代闭环
        for (int i = 0; i < kMaxIterateCount; ++i) {
            // 计算预测的时间点 = 子弹飞行时间 + 系统响应延迟
            double total_predict_time = current_fly_time + config.shoot_delay;
            auto t_target             = snapshot.time_stamp()
                + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(total_predict_time));
            // A. 预测该时刻所有装甲板的位置
            auto predicted_armors = snapshot.predicted_armors(t_target);
            auto predicted_ekf_x  = snapshot.predict_at(t_target);

            best_armor_opt = aim_point_chooser.choose_armor(predicted_armors, predicted_ekf_x);
            if (!best_armor_opt) return std::nullopt;

            // C. 针对这块选中的装甲板，解算带阻力的弹道
            // ROS 坐标：x前 y左 z上，pitch 为绕 y 轴右手旋转（枪口抬起为正）。
            // 因此旋转矩阵 R_y(pitch) = [[cos p,0,sin p],[0,1,0],[-sin p,0,cos p]]
            double dynamic_offset_x = config.shoot_offset_x * std::cos(iter_pitch)
                + config.shoot_offset_z * std::sin(iter_pitch);
            double dynamic_offset_y = config.shoot_offset_y; // pitch 旋转对 y 不影响
            double dynamic_offset_z = -config.shoot_offset_x * std::sin(iter_pitch)
                + config.shoot_offset_z * std::cos(iter_pitch);

            auto const& translation = best_armor_opt->translation;
            double target_d =
                std::sqrt((translation.x - dynamic_offset_x) * (translation.x - dynamic_offset_x)
                    + (translation.y - dynamic_offset_y) * (translation.y - dynamic_offset_y));
            auto target_z = translation.z - dynamic_offset_z;

            auto solution_params       = fire_control::TrajectorySolution::TrajectoryParams {};
            solution_params.k          = config.k;
            solution_params.g          = config.g;
            solution_params.bias_scale = config.bias_scale;

            auto solution           = TrajectorySolution {};
            solution.input.v0       = config.v_initial;
            solution.input.target_d = target_d;
            solution.input.target_h = target_z;
            solution.params         = solution_params;

            auto result = solution.solve();
            if (!result) return std::nullopt;

            auto time_error   = std::abs(result->fly_time - current_fly_time);
            current_fly_time  = result->fly_time;
            iter_pitch        = result->pitch;
            trajectory_result = *result;
            horizon_distance  = target_d;

            if (time_error < kMaxFlyTimeThreold) break;
        }

        auto final_yaw = std::atan2(best_armor_opt->translation.y, best_armor_opt->translation.x);

        return Result {
            .pitch = trajectory_result.pitch, .yaw = final_yaw, .horizon_distance = horizon_distance
        };
    }
};

FireControl::FireControl() noexcept
    : pimpl { std::make_unique<Impl>() } { }
FireControl::~FireControl() noexcept = default;

auto FireControl::initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
    return pimpl->initialize(yaml);
}

auto FireControl::solve(const predictor::Snapshot& snapshot) -> std::optional<Result> {
    return pimpl->solve(snapshot);
}
