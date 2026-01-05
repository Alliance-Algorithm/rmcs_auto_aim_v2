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
        double v_initial;   // m/s
        double shoot_delay; // s

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

    auto solve(const predictor::Snapshot& snapshot) -> auto {
        // 1. 初始猜测：以当前目标中心距离估算初次飞行时间
        auto state = snapshot.ekf_x();
        auto x = state[0], y = state[2], z = state[4];
        auto distance = std::sqrt(x * x + y * y + z * z);

        auto current_fly_time = distance / config.v_initial;

        auto best_armor_opt = std::optional<Armor3D> {};
        auto trajectory     = TrajectorySolution::Output {};

        // 2. 迭代闭环
        for (int i = 0; i < kMaxIterateCount; ++i) {
            // 计算预测的时间点 = 子弹飞行时间 + 系统响应延迟
            double total_predict_time = current_fly_time + config.shoot_delay;
            auto t_target             = snapshot.time_stamp()
                + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(total_predict_time));
            // A. 预测该时刻所有装甲板的位置
            auto predicted_armors = snapshot.predicted_armors(t_target);
        }
    }
};

FireControl::FireControl() noexcept
    : pimpl { std::make_unique<Impl>() } { }
FireControl::~FireControl() noexcept = default;

auto FireControl::initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
    return pimpl->initialize(yaml);
}
