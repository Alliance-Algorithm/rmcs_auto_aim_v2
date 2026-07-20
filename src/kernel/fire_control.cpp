#include "fire_control.hpp"

#include "module/fire_control/shoot_evaluator.hpp"
#include "module/fire_control/trajectory_solution.hpp"
#include "utility/logging/printer.hpp"
#include "utility/math/angle.hpp"
#include "utility/repeat.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <vector>

using namespace rmcs::kernel;
using namespace rmcs::util;

struct FireController::Impl {
    Config config;

    State state;

    std::int8_t last_idx = -1;
    DeviceId last_device = DeviceId::UNKNOWN;

    Repeat rune_attack_actions {
        Repeat::Action { "idle", 0.4 },
        Repeat::Action { "shoot", 0.2 },
    };

    Printer logging { "fire" };
    std::unique_ptr<ShootEvaluator> shoot_evaluator;

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

    /// @brief 预瞄边界点：来板一侧窗口边界在装甲板轨道上的位置。
    /// 边界角为目标本体角度（与 yaw_angle 同帧），z 与半径取来板实际值。
    static constexpr auto boundary_point(
        const Point3d& center, const AimPoints& aimpoints, double offset) {

        // 来板：与边界同侧且 |delta| 最小的装甲板
        const AimPoint* incoming = nullptr;

        auto best = std::numeric_limits<double>::max();
        for (const auto& armor : aimpoints) {
            const auto delta = util::normalize_angle(yaw_angle(center, armor));
            if (((offset > 0) == (delta > 0)) && std::abs(delta) < best) {
                best     = std::abs(delta);
                incoming = &armor;
            }
        }

        const auto radius = incoming
            ? std::hypot(incoming->x - center.x, incoming->y - center.y)
            : std::hypot(aimpoints[0].x - center.x, aimpoints[0].y - center.y);
        const auto z      = incoming ? incoming->z : center.z;
        const auto angle  = std::atan2(-center.y, -center.x) + offset;

        return Point3d {
            center.x + radius * std::cos(angle),
            center.y + radius * std::sin(angle),
            z,
        };
    }

    explicit Impl(const Config& cfg)
        : config { cfg } {
        if (!(config.yaw_tolerance > 0.0)) {
            throw std::runtime_error { "FireControllerV2: yaw_tolerance must be > 0" };
        }
        if (!(config.pitch_tolerance > 0.0)) {
            throw std::runtime_error { "FireControllerV2: pitch_tolerance must be > 0" };
        }
        if (!(config.window_hysteresis >= 0.0 && config.window_hysteresis < 1.0)) {
            throw std::runtime_error { "FireControllerV2: window_hysteresis must be in [0, 1)" };
        }
        if (!(config.degraded_angle_speed >= 0.0)) {
            throw std::runtime_error { "FireControllerV2: degraded_angle_speed must be >= 0" };
        }

        config.offset_yaw    = util::deg2rad(config.offset_yaw);
        config.offset_pitch  = util::deg2rad(config.offset_pitch);
        config.attack_window = util::deg2rad(config.attack_window);

        shoot_evaluator = std::make_unique<ShootEvaluator>(ShootEvaluator::Config {
            .yaw_tolerance          = config.yaw_tolerance,
            .pitch_tolerance        = config.pitch_tolerance,
            .require_stable_command = config.require_stable_command,
            .is_lazy_gimbal         = config.is_lazy_gimbal,
        });
    }

    /// @brief 锯齿波动态攻击窗口计算。
    /// 将四板目标的 Yaw 跟踪信号建模为锯齿波：每块装甲板扫过视野时 Yaw 近似线性变化，
    /// 切换板时瞬时跳变。在云台最大速度和最大加速度约束下计算匀速跟踪段窗口。
    /// 若目标角速度超过 degraded_angle_speed，则判定为退化，下游改为瞄准中心，
    /// 并按装甲板扫过弹道射线决定是否发弹。
    ///
    /// 预瞄时间采用时间最优的 bang-bang（加速度主导）或梯形曲线（速度饱和）建模。
    /// 跟踪角速度阈值由云台视角的装甲板轨道真实角跨度计算：2·arcsin(r·sin45°/L)。
    ///
    /// 返回值经几何反解转为目标本体角度半幅，与 yaw_angle() 的 delta 同帧。
    ///
    /// @param  trackable  跟踪目标
    /// @return 目标本体角度半幅 (rad)，供下游与配置窗口取 min 夹紧；
    ///         inf 表示不限制窗口，NaN 表示退化（下游改为瞄准中心）
    auto get_attack_window(const Trackable& trackable) const -> double {
        constexpr auto kUnlimited = std::numeric_limits<double>::infinity();
        constexpr auto kDegraded  = std::numeric_limits<double>::quiet_NaN();

        const auto omega = std::abs(trackable.get_rotation_speed());
        if (omega < 1e-1) {
            return kUnlimited;
        }

        // 退化判定：目标角速度超过配置阈值
        if (omega > config.degraded_angle_speed) {
            return kDegraded;
        }

        if (state.max_yaw_vel <= 0.0 || state.max_yaw_acc <= 0.0) {
            return kUnlimited;
        }

        const auto center = trackable.get_direction();

        // 几何模型：云台-目标中心-装甲板中心，夹角 45°
        auto radius   = double { };
        auto distance = double { };
        auto stroke   = double { };
        {
            const auto aimpoints = trackable.get_aimpoints();

            radius   = std::hypot(aimpoints[0].x - center.x, aimpoints[0].y - center.y);
            distance = std::hypot(center.x, center.y);

            if (radius <= 0.0 || distance <= 0.0) {
                return kUnlimited;
            }

            constexpr auto kHalfAngle = std::numbers::pi / 4.0;

            const auto L = std::sqrt(distance * distance + radius * radius
                - 2.0 * distance * radius * std::cos(kHalfAngle));
            const auto A = std::asin(radius * std::sin(kHalfAngle) / L);

            stroke = 2.0 * A;
        }

        // 锯齿波参数
        const auto cycle_time = std::numbers::pi / (2.0 * omega);
        const auto v_track    = stroke / cycle_time;

        const auto v_max = state.max_yaw_vel * 0.8;
        const auto a_max = state.max_yaw_acc * 0.8;

        // 预瞄时间计算
        auto stable_window = double { };
        {
            const auto sqrt_stroke_acc = std::sqrt(stroke * a_max);
            if (sqrt_stroke_acc <= v_max + v_track) {
                stable_window = stroke - 2.0 * v_track * std::sqrt(stroke / a_max);
            } else {
                stable_window =
                    v_max * stroke / (v_max + v_track) - v_track * (v_max + v_track) / a_max;
            }
        }

        // 无稳定跟踪段时窗口收敛为零，不再触发退化
        stable_window = std::max(stable_window, 0.0);

        // 将云台视角半幅转换回目标本体角度半幅
        auto half_window = double { };
        {
            const auto phi     = stable_window * 0.5;
            const auto sin_sum = (distance / radius) * std::sin(phi);
            if (sin_sum >= 1.0) {
                half_window = std::numeric_limits<double>::infinity();
            } else {
                half_window = std::asin(sin_sum) - phi;
            }
        }

        return half_window;
    }

    /// @brief 装甲板目标评估。
    /// @return (aim_index, pre_aim_offset)
    ///         - aim_index: 装甲板索引；
    ///           int8_min 表示预瞄窗口边界，int8_max 表示退化（由 aim() 瞄准中心）
    ///         - pre_aim_offset: 预瞄边界角（带符号，rad，目标本体角度，与 yaw_angle 同帧），
    ///           仅 aim_index 为 int8_min 时有效
    auto evaluate_armor(const Trackable& trackable, bool use_pre_aim = true)
        -> std::tuple<std::int8_t, double> {

        const auto aimpoints = trackable.get_aimpoints();
        const auto omega     = trackable.get_rotation_speed();

        if (aimpoints.size() == 1) return { 0, 0.0 };

        // 目标身份变化时重置滞回状态
        if (use_pre_aim && trackable.id() != last_device) {
            last_device = trackable.id();
            last_idx    = -1;
        }

        // 窗口计算
        auto half_window = double { };

        if (use_pre_aim) {
            auto dynamic_window = std::numeric_limits<double>::infinity();
            if (aimpoints.size() == 4) {
                dynamic_window = get_attack_window(trackable);

                // 退化：直接返回哨兵值，由 aim() 瞄准中心
                if (std::isnan(dynamic_window)) {
                    last_idx = -1;
                    return { std::numeric_limits<std::int8_t>::max(), 0.0 };
                }
            }
            half_window = std::min(config.attack_window / 2, dynamic_window);
        } else {
            half_window = config.attack_window / 2;
        }

        // 施密特滞回：进入阈值收紧，退出阈值放宽；raw 评估保持无状态
        const auto enter_window =
            use_pre_aim ? half_window * (1.0 - config.window_hysteresis) : half_window;
        const auto exit_window = half_window * (1.0 + config.window_hysteresis);

        // 装甲板选择
        const auto length = static_cast<std::int8_t>(aimpoints.size());
        if (use_pre_aim && last_idx >= 0 && last_idx < length) {
            const auto delta =
                util::normalize_angle(yaw_angle(trackable.get_direction(), aimpoints[last_idx]));
            if (std::abs(delta) <= exit_window) {
                return { last_idx, 0.0 };
            }
        }

        auto best_index = std::optional<std::int8_t> { };
        auto best_delta = std::numeric_limits<double>::max();

        for (auto i = std::size_t { 0 }; i < aimpoints.size(); ++i) {
            const auto delta =
                util::normalize_angle(yaw_angle(trackable.get_direction(), aimpoints[i]));
            const auto abs_delta = std::abs(delta);

            if (abs_delta <= enter_window && abs_delta < best_delta) {
                best_delta = abs_delta;
                best_index = static_cast<std::int8_t>(i);
            }
        }

        if (best_index) {
            if (use_pre_aim) last_idx = *best_index;
            return { *best_index, 0.0 };
        }

        // 预瞄：窗口内无板，瞄准来板一侧的窗口边界，减少云台跟踪量
        if (use_pre_aim) {
            last_idx            = -1;
            const auto boundary = (omega < 0) ? +half_window : -half_window;
            return { std::numeric_limits<std::int8_t>::min(), boundary };
        }
        return { -1, 0.0 };
    }

    /// @brief 能量机关符叶评估。
    /// @return (aim_index, idle)
    ///         - aim_index: 符叶索引；int8_max 表示视野局限，由 aim() 瞄准中心
    ///         - idle:      时序空闲期，继续瞄准符叶但抑制发射
    auto evaluate_blade(const Trackable& trackable) -> std::tuple<std::int8_t, bool> {
        const auto aimpoints = trackable.get_aimpoints();
        const auto indices   = aimpoints.valid_indices();
        if (!indices.empty()) {
            if (rune_attack_actions.started() == false) {
                rune_attack_actions.start();
            }

            const auto tag = rune_attack_actions.tick();
            if (tag == "idle") {
                return { static_cast<std::int8_t>(indices[0]), true };
            } else {
                return { static_cast<std::int8_t>(indices[0]), false };
            }
        }

        // 视野局限，使其瞄准中心
        return { std::numeric_limits<std::int8_t>::max(), true };
    }

    auto aim(const Trackable& trackable) -> std::optional<Aimed> {

        const auto direction = trackable.get_direction();
        const auto distance  = std::sqrt(
            direction.x * direction.x + direction.y * direction.y + direction.z * direction.z);

        const auto fly_time  = distance / config.bullet_speed;
        const auto increment = config.shoot_delay
            + std::chrono::duration<double>(state.timestamp - trackable.get_timestamp()).count();

        auto aim_selected   = std::int8_t { -1 };
        auto pre_aim        = false;
        auto pre_aim_offset = double { };
        {
            auto future = trackable.clone();
            future->jump_into(fly_time + increment);
            if (trackable.id() == DeviceId::RUNE) {
                std::tie(aim_selected, pre_aim) = evaluate_blade(*future);
            } else {
                std::tie(aim_selected, pre_aim_offset) = evaluate_armor(*future);
                pre_aim = (aim_selected == std::numeric_limits<std::int8_t>::min());
            }
        }
        if (aim_selected == -1) return std::nullopt;

        const auto degraded     = (aim_selected == std::numeric_limits<std::int8_t>::max());
        const auto aim_boundary = (aim_selected == std::numeric_limits<std::int8_t>::min());

        constexpr auto kMaxIterate = std::size_t { 5 };
        constexpr auto kEpsilon    = 0.001;

        auto attack = Point3d { };
        auto center = Point3d { };
        auto yaw    = double { };
        auto pitch  = double { };

        auto aimpoints = AimPoints { };
        auto new_time  = double { fly_time };

        auto solution = TrajectorySolution { };
        for (auto i = std::size_t { 0 }; i < kMaxIterate; ++i) {
            auto clone = trackable.clone();
            clone->jump_into(new_time + increment);

            center = clone->get_direction();
            if (degraded) {
                aimpoints = clone->get_aimpoints();
                attack    = center;
            } else if (aim_boundary) {
                attack = boundary_point(center, clone->get_aimpoints(), pre_aim_offset);
            } else {
                attack = clone->get_aimpoints()[aim_selected];
            }

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
        yaw   = yaw + config.offset_yaw;
        pitch = pitch + config.offset_pitch;

        // 退化发射评估：任一装甲板扫过弹道射线（横向偏移不超过板半宽）即允许发弹
        const auto armor_on_ray = [&](const Point3d& center, const AimPoints& aimpoints) {
            const auto center_distance = std::hypot(center.x, center.y);
            if (center_distance <= 1e-6) return false;

            return std::ranges::any_of(aimpoints, [&](const AimPoint& armor) {
                // 排除背面板：仅当装甲板位于朝向云台一侧
                if (std::abs(yaw_angle(center, armor)) >= std::numbers::pi / 2.0) {
                    return false;
                }

                // 板心到过中心射线的垂直距离（XY 平面）
                const auto offset =
                    std::abs(armor.x * center.y - armor.y * center.x) / center_distance;
                return offset <= config.yaw_tolerance;
            });
        };

        // 射击评估
        auto should_shoot = true;
        /*  */ if (trackable.id() == DeviceId::RUNE) {
            should_shoot = !degraded && !pre_aim;
        } else if (degraded) {
            should_shoot = armor_on_ray(center, aimpoints);
        } else {
            const auto cmd = ShootEvaluator::Command {
                .yaw    = yaw,
                .pitch  = pitch,
                .center = center,
                .armor  = attack,
            };
            if (pre_aim && !config.attack_preaim) {
                should_shoot = false;
            } else {
                should_shoot = shoot_evaluator->evaluate(cmd, state.yaw, state.pitch);
            }
        }

        const auto make_target = [](double yaw, double pitch) {
            return Direction3d {
                +std::cos(pitch) * std::cos(yaw),
                +std::cos(pitch) * std::sin(yaw),
                -std::sin(pitch),
            };
        };
        return Aimed {
            .aim_yaw = yaw,
            .pitch   = pitch,
            .shoot   = should_shoot,
            .pre_aim = pre_aim,
            .target  = make_target(yaw, pitch),
            .center  = center,
            .attack  = attack,
        };
    }
};

FireController::FireController(const Config& config)
    : pimpl { std::make_unique<Impl>(config) } { }

FireController::~FireController() noexcept = default;

auto FireController::update(State state) -> void { pimpl->state = state; }

auto FireController::aim(const Trackable& trackable) -> std::optional<Aimed> {
    return pimpl->aim(trackable);
}
