#include "fire_control.hpp"

#include "module/fire_control/shoot_evaluator.hpp"
#include "module/fire_control/trajectory_solution.hpp"
#include "utility/logging/printer.hpp"
#include "utility/math/angle.hpp"
#include "utility/serializable.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <memory>
#include <optional>
#include <vector>

using namespace rmcs::kernel;
using namespace rmcs::util;

struct FireControllerV2::Impl {

    struct Config : Serializable {
        double bullet_speed;
        double shoot_delay;
        double offset_yaw { 0.0 };
        double offset_pitch { 0.0 };

        std::array<double, 2> attack_window { 20.0, 20.0 };

        double max_vel { 0.0 };
        double max_acc { 0.0 };

        double yaw_tolerance { 0.07 };
        double pitch_tolerance { 0.04 };
        bool require_stable_command { true };
        bool is_lazy_gimbal { false };

        static constexpr std::tuple metas {
            // clang-format off
            &Config::bullet_speed, "bullet_speed",
            &Config::shoot_delay, "shoot_delay",
            &Config::offset_yaw, "offset_yaw",
            &Config::offset_pitch, "offset_pitch",
            &Config::attack_window, "attack_window",
            &Config::max_vel, "max_vel",
            &Config::max_acc, "max_acc",
            &Config::yaw_tolerance, "yaw_tolerance",
            &Config::pitch_tolerance, "pitch_tolerance",
            &Config::require_stable_command, "require_stable_command",
            &Config::is_lazy_gimbal, "is_lazy_gimbal",
            // clang-format on
        };
    } config;

    std::unique_ptr<ShootEvaluator> shoot_evaluator;

    double cached_yaw   = kNaN;
    double cached_pitch = kNaN;
    Timestamp cached_stamp;
    std::optional<std::size_t> last_armor_idx;

    Printer logging { "fire" };

    static constexpr auto yaw_angle(const Point3d& center, const Point3d& armor) {
        const auto v1_x = -center.x;
        const auto v1_y = -center.y;
        const auto v2_x = -center.x + armor.x;
        const auto v2_y = -center.y + armor.y;

        const auto dot  = v1_x * v2_x + v1_y * v2_y;
        const auto len1 = std::sqrt(v1_x * v1_x + v1_y * v1_y);
        const auto len2 = std::sqrt(v2_x * v2_x + v2_y * v2_y);

        const auto denom = len1 * len2;
        if (denom < 1e-6) return 0.0;

        const auto cos_theta = std::clamp(dot / denom, -1.0, +1.0);
        const auto abs_angle = std::acos(cos_theta);
        const auto cross_z   = v1_x * v2_y - v1_y * v2_x;

        return (cross_z >= 0.0) ? abs_angle : -abs_angle;
    }

    // @return 序号, 是否预瞄
    static auto get_aimed_id(const Trackable& trackable, std::tuple<double, double> window)
        -> std::tuple<std::int8_t, bool> {

        const auto aimpoints = trackable.get_aimpoints();
        const auto omega     = trackable.get_rotation_speed();

        auto best_idx   = std::optional<std::size_t> { };
        auto best_delta = std::numeric_limits<double>::max();
        auto next_idx   = std::optional<std::size_t> { };
        auto next_delta = std::numeric_limits<double>::max();

        for (std::size_t i = 0; i < aimpoints.size(); ++i) {
            const auto delta =
                util::normalize_angle(yaw_angle(trackable.get_direction(), aimpoints[i]));
            const auto abs_delta = std::abs(delta);

            // 窗口内：找最接近中心的
            const auto threshold = (delta > 0) ? std::get<0>(window) : std::get<1>(window);
            if (abs_delta <= threshold && abs_delta < best_delta) {
                best_delta = abs_delta;
                best_idx   = i;
            }

            // 窗口外：根据旋转方向选下一个即将进入的
            const auto dominated = (omega < 0) ? (delta > 0) : (delta < 0);
            if (dominated && abs_delta < next_delta) {
                next_delta = abs_delta;
                next_idx   = i;
            }
        }

        return { best_idx ? *best_idx : next_idx.value_or(-1), !best_idx.has_value() };
    }

    static auto get_cycle_time(const Trackable& trackable) {
        const auto omega = std::abs(trackable.get_rotation_speed());
        if (omega < 1e-6) return 0.0;

        const auto n = static_cast<double>(trackable.get_aimpoints().size());
        if (n < 1.0) return 0.0;

        return 2.0 * std::numbers::pi / (n * omega);
    }

    explicit Impl(const YAML::Node& yaml) {
        if (const auto ret = config.serialize(yaml); !ret.has_value()) {
            throw std::runtime_error { std::format("FireControllerV2: {}", ret.error()) };
        }

        if (!(config.yaw_tolerance > 0.0)) {
            throw std::runtime_error { "FireControllerV2: yaw_tolerance must be > 0" };
        }

        if (!(config.pitch_tolerance > 0.0)) {
            throw std::runtime_error { "FireControllerV2: pitch_tolerance must be > 0" };
        }

        config.offset_yaw       = util::deg2rad(config.offset_yaw);
        config.offset_pitch     = util::deg2rad(config.offset_pitch);
        config.attack_window[0] = util::deg2rad(config.attack_window[0]);
        config.attack_window[1] = util::deg2rad(config.attack_window[1]);
        config.max_vel          = util::deg2rad(config.max_vel);
        config.max_acc          = util::deg2rad(config.max_acc);

        shoot_evaluator = std::make_unique<ShootEvaluator>(ShootEvaluator::Config {
            .yaw_tolerance          = config.yaw_tolerance,
            .pitch_tolerance        = config.pitch_tolerance,
            .require_stable_command = config.require_stable_command,
            .is_lazy_gimbal         = config.is_lazy_gimbal,
        });
    }

    auto get_attack_window(const Trackable& trackable) const -> std::tuple<double, double> {
        if (config.max_vel <= 0.0 || config.max_acc <= 0.0) {
            return { config.attack_window[0], config.attack_window[1] };
        }

        const auto cycle_time = get_cycle_time(trackable);
        if (cycle_time == 0.0) {
            return { config.attack_window[0], config.attack_window[1] };
        }

        const auto period    = cycle_time;
        const auto vel_limit = config.max_vel * period / (2.0 * std::numbers::pi);
        const auto acc_limit =
            config.max_acc * period * period / (4.0 * std::numbers::pi * std::numbers::pi);

        const auto physical_limit = std::min(vel_limit, acc_limit);

        return {
            std::min(config.attack_window[0], physical_limit),
            std::min(config.attack_window[1], physical_limit),
        };
    }

    auto aim(const Trackable& trackable) const -> std::optional<Aimed> {

        const auto direction = trackable.get_direction();
        const auto distance  = std::sqrt(
            direction.x * direction.x + direction.y * direction.y + direction.z * direction.z);

        const auto fly_time  = distance / config.bullet_speed;
        const auto increment = config.shoot_delay
            + std::chrono::duration<double>(cached_stamp - trackable.get_timestamp()).count();

        auto selected = std::int8_t { -1 };
        auto pre_aim  = false;
        {
            auto future = trackable.clone();
            future->jump_into(fly_time + increment);

            const auto window = get_attack_window(*future);

            std::tie(selected, pre_aim) = get_aimed_id(*future, window);
        }

        if (selected >= 0) {
            constexpr auto kMaxIterate = std::size_t { 5 };
            constexpr auto kEpsilon    = 0.001;

            // 迭代收敛飞行时间
            auto attack = Point3d { };
            auto center = Point3d { };
            auto yaw    = double { };
            auto pitch  = double { };

            auto new_time = double { fly_time };

            auto solution = TrajectorySolution { };
            for (std::size_t i = 0; i < kMaxIterate; ++i) {
                auto clone = trackable.clone();
                clone->jump_into(new_time + increment);

                const auto aimpoints = clone->get_aimpoints();

                attack = aimpoints[selected];
                center = clone->get_direction();

                solution.input.v0    = config.bullet_speed;
                solution.input.point = attack;

                const auto result = solution.solve();
                if (!result) return std::nullopt;

                const auto prev = new_time;

                new_time = result->fly_time;
                yaw      = result->yaw;
                pitch    = result->pitch;

                if (std::abs(new_time - prev) < kEpsilon) break;
            }

            // 偏置校正
            yaw   = util::normalize_angle(yaw + config.offset_yaw);
            pitch = pitch + config.offset_pitch;

            // 射击评估
            const auto shoot = shoot_evaluator->evaluate(
                ShootEvaluator::Command {
                    .yaw    = yaw,
                    .pitch  = pitch,
                    .center = center,
                    .armor  = attack,
                },
                cached_yaw, cached_pitch);

            return Aimed {
                .yaw     = yaw,
                .pitch   = pitch,
                .shoot   = shoot,
                .pre_aim = pre_aim,
                .center  = center,
                .attack  = attack,
            };
        }
        return std::nullopt;
    }
};

FireControllerV2::FireControllerV2(const YAML::Node& yaml)
    : pimpl { std::make_unique<Impl>(yaml) } { }

FireControllerV2::~FireControllerV2() noexcept = default;

auto FireControllerV2::update(Timestamp timestamp, double yaw, double pitch) -> void {
    pimpl->cached_stamp = timestamp;
    pimpl->cached_yaw   = yaw;
    pimpl->cached_pitch = pitch;
}

auto FireControllerV2::aim(const Trackable& trackable) -> std::optional<Aimed> {
    return pimpl->aim(trackable);
}
