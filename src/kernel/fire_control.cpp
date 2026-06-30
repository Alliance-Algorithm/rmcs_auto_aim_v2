#include "fire_control.hpp"

#include <cmath>
#include <format>
#include <memory>
#include <optional>

#include "module/fire_control/shoot_evaluator.hpp"
#include "module/fire_control/trajectory_solution.hpp"
#include "utility/math/angle.hpp"
#include "utility/serializable.hpp"

using namespace rmcs::kernel;
using namespace rmcs::fire_control;
using namespace rmcs::util;

struct FireControllerV2::Impl {

    struct Config : Serializable {
        double bullet_speed;
        double shoot_delay;
        double offset_yaw { 0.0 };
        double offset_pitch { 0.0 };

        std::array<double, 2> attack_window { 20.0, 20.0 };

        static constexpr std::tuple metas {
            // clang-format off
            &Config::bullet_speed, "bullet_speed",
            &Config::shoot_delay, "shoot_delay",
            &Config::offset_yaw, "offset_yaw",
            &Config::offset_pitch, "offset_pitch",
            &Config::attack_window, "attack_window",
            // clang-format on
        };
    } config;

    ShootEvaluator shoot_evaluator;

    double cached_yaw   = kNaN;
    double cached_pitch = kNaN;
    Timestamp cached_stamp;
    std::optional<std::size_t> last_armor_id;

    explicit Impl(const YAML::Node& yaml) {
        if (const auto ret = config.serialize(yaml); !ret.has_value()) {
            throw std::runtime_error { std::format("FireControllerV2: {}", ret.error()) };
        }

        config.offset_yaw       = util::deg2rad(config.offset_yaw);
        config.offset_pitch     = util::deg2rad(config.offset_pitch);
        config.attack_window[0] = util::deg2rad(config.attack_window[0]);
        config.attack_window[1] = util::deg2rad(config.attack_window[1]);

        if (const auto ret = shoot_evaluator.configure_yaml(yaml["shoot_evaluator"]);
            !ret.has_value()) {
            throw std::runtime_error {
                std::format("FireControllerV2 shoot_evaluator: {}", ret.error()),
            };
        }
    }

    auto aim(const Trackable& trackable) -> std::optional<Aimed> {
        constexpr auto kMaxIterate = std::size_t { 5 };
        constexpr auto kEpsilon    = 0.001;

        // 粗估飞行时间
        const auto center = trackable.direction().make<Eigen::Vector3d>();

        auto fly_time = center.norm() / config.bullet_speed;

        // 预测采样 → 选定装甲板

        auto armor_id = std::optional<std::size_t> { };
        {
            auto clone = trackable.clone();
            clone->jump_into(config.shoot_delay + fly_time);

            const auto aimpoints = clone->aimpoints();
            const auto center    = clone->direction().make<Eigen::Vector3d>();
            const auto cam_dir   = std::atan2(center.y(), center.x());

            // 先检查上一帧选中的板是否还在窗口内
            if (last_armor_id && *last_armor_id < aimpoints.size()) {
                const auto pt    = aimpoints[*last_armor_id].make<Eigen::Vector3d>();
                const auto dir   = pt - center;
                const auto delta = util::normalize_angle(std::atan2(-dir.y(), -dir.x()) - cam_dir);

                const auto abs_delta = std::abs(delta);
                const auto threshold =
                    (delta > 0) ? config.attack_window[0] : config.attack_window[1];
                if (abs_delta <= threshold) {
                    armor_id = last_armor_id;
                }
            }

            // 上一块板出窗了（或没有上一帧），遍历所有候选重新选
            if (!armor_id.has_value()) {
                auto best_id    = std::optional<size_t> { };
                auto best_delta = std::numeric_limits<double>::max();

                for (std::size_t i = 0; i < aimpoints.size(); ++i) {
                    const auto pt  = aimpoints[i].make<Eigen::Vector3d>();
                    const auto dir = pt - center;
                    const auto delta =
                        util::normalize_angle(std::atan2(-dir.y(), -dir.x()) - cam_dir);

                    const auto abs_delta = std::abs(delta);
                    const auto threshold =
                        (delta > 0) ? config.attack_window[0] : config.attack_window[1];
                    if (abs_delta > threshold) continue;

                    if (abs_delta < best_delta) {
                        best_delta = abs_delta;
                        best_id    = i;
                    }
                }
                armor_id = best_id;
            }

            if (!armor_id.has_value()) {
                last_armor_id.reset();
                return std::nullopt;
            }
        }

        last_armor_id = armor_id;

        // 迭代收敛飞行时间

        auto attack = Eigen::Vector3d { };
        auto yaw    = double { };
        auto pitch  = double { };

        for (size_t i = 0; i < kMaxIterate; ++i) {
            auto clone = trackable.clone();
            clone->jump_into(config.shoot_delay + fly_time);

            const auto aimpoints = clone->aimpoints();
            if (*armor_id >= aimpoints.size()) return std::nullopt;

            attack = aimpoints[*armor_id].make<Eigen::Vector3d>();

            auto solution = TrajectorySolution { };

            solution.input.v0    = config.bullet_speed;
            solution.input.point = attack;

            const auto result = solution.solve();
            if (!result) return std::nullopt;

            const auto prev_fly_time = fly_time;

            fly_time = result->fly_time;
            yaw      = result->yaw;
            pitch    = result->pitch;

            if (std::abs(fly_time - prev_fly_time) < kEpsilon) break;
        }

        // 偏置校正

        yaw   = util::normalize_angle(yaw + config.offset_yaw);
        pitch = pitch + config.offset_pitch;

        // 射击评估

        const auto shoot = shoot_evaluator.evaluate(
            {
                .yaw    = yaw,
                .pitch  = pitch,
                .center = center,
                .attack = attack,
            },
            {
                .timestamp = cached_stamp,
                .yaw       = cached_yaw,
                .pitch     = cached_pitch,
            });

        return Aimed {
            .aim_yaw = yaw,
            .raw_yaw = yaw,
            .pitch   = pitch,
            .shoot   = shoot,
            .center  = Point3d { center },
            .attack  = Point3d { attack },
        };
    }
};

FireControllerV2::FireControllerV2(const YAML::Node& yaml)
    : pimpl { std::make_unique<Impl>(yaml) } { }

FireControllerV2::~FireControllerV2() noexcept = default;

auto FireControllerV2::update(double yaw, double pitch) -> void {
    pimpl->cached_yaw   = yaw;
    pimpl->cached_pitch = pitch;
}

auto FireControllerV2::update(Timestamp timestamp) -> void { pimpl->cached_stamp = timestamp; }

auto FireControllerV2::aim(const Trackable& trackable) -> std::optional<Aimed> {
    return pimpl->aim(trackable);
}
