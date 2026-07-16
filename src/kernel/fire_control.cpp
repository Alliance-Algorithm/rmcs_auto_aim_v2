#include "fire_control.hpp"

#include "module/fire_control/shoot_evaluator.hpp"
#include "module/fire_control/trajectory_solution.hpp"
#include "utility/logging/printer.hpp"
#include "utility/math/angle.hpp"
#include "utility/repeat.hpp"
#include "utility/serializable.hpp"

#include <algorithm>
#include <cmath>
#include <format>
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

    Repeat single_rune_actions {
        Repeat::Action { "idle", 0.4 },
        Repeat::Action { "shoot", 0.2 },
    };
    Repeat multiple_rune_actions {
        Repeat::Action { "1-idle", 0.4 },
        Repeat::Action { "1-shoot", 0.2 },
        Repeat::Action { "2-idle", 0.4 },
        Repeat::Action { "2-shoot", 0.2 },
    };
    AimPoints last_aimpoints;

    Printer logging { "fire" };
    std::unique_ptr<ShootEvaluator> shoot_evaluator;

    static constexpr auto make_target(double yaw, double pitch) -> Direction3d {
        return Direction3d {
            +std::cos(pitch) * std::cos(yaw),
            +std::cos(pitch) * std::sin(yaw),
            -std::sin(pitch),
        };
    }

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
        if (!(config.window_hysteresis >= 0.0 && config.window_hysteresis < 1.0)) {
            throw std::runtime_error { "FireControllerV2: window_hysteresis must be in [0, 1)" };
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
    /// 切换板时瞬时跳变。在云台最大速度和最大加速度约束下判断是否具备匀速跟踪段。
    /// 若无，则判定为退化，下游改为瞄准中心射击。
    ///
    /// 预瞄时间采用时间最优的 bang-bang（加速度主导）或梯形曲线（速度饱和）建模。
    /// 跟踪角速度阈值由云台视角的装甲板轨道真实角跨度计算：2·arcsin(r·sin45°/L)。
    ///
    /// 返回值经几何反解转为目标本体角度半幅，与 yaw_angle() 的 delta 同帧。
    ///
    /// @param  trackable  跟踪目标
    /// @return (window_half, degraded)
    ///         - window_half: 目标本体角度半幅 (rad)，供下游与配置窗口取 min 夹紧
    ///         - degraded:    是否退化
    auto get_attack_window(const Trackable& trackable) const -> std::tuple<double, bool> {
        const auto omega = std::abs(trackable.get_rotation_speed());
        if (omega < 1e-6) {
            return { std::numeric_limits<double>::infinity(), false };
        }
        if (state.max_yaw_vel <= 0.0 || state.max_yaw_acc <= 0.0) {
            return { std::numeric_limits<double>::infinity(), false };
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
                return { std::numeric_limits<double>::infinity(), false };
            }

            constexpr auto kHalfAngle = std::numbers::pi / 4.0;

            const auto L = std::sqrt(distance * distance + radius * radius
                - 2.0 * distance * radius * std::cos(kHalfAngle));
            const auto A = std::asin(radius * std::sin(kHalfAngle) / L);

            stroke = 2.0 * A;
        }

        // 退化窗口：yaw_tolerance 距离量对应的目标本体角度半幅
        const auto degraded_window = [&](double radius) -> double {
            const auto ratio = config.yaw_tolerance / radius;
            if (ratio >= 1.0) {
                return std::numeric_limits<double>::infinity();
            }
            return std::asin(ratio);
        };

        // 锯齿波参数
        const auto cycle_time = std::numbers::pi / (2.0 * omega);
        const auto v_track    = stroke / cycle_time;

        const auto v_max = state.max_yaw_vel;
        const auto a_max = state.max_yaw_acc;

        // 第一重退化：匀速跟踪段斜率超过云台最大速度
        if (v_track > v_max) {
            return { degraded_window(radius), true };
        }

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

        // 第二重退化：预瞄时间占满整个周期，无稳定跟踪段
        if (stable_window <= 0.0) {
            return { degraded_window(radius), true };
        }

        // 将云月视角半幅转换回目标本体角度半幅
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

        return { half_window, false };
    }

    auto evaluate_aimed(const Trackable& trackable, bool use_pre_aim = true)
        -> std::tuple<std::int8_t, bool> {

        auto aimpoints = trackable.get_aimpoints();
        auto omega     = trackable.get_rotation_speed();

        if (aimpoints.size() == 1) {
            return { 0, false };
        }

        // 能量机关
        if (aimpoints.size() == 5) {
            if (!aimpoints.same_valid(last_aimpoints)) {
                single_rune_actions.stop();
                multiple_rune_actions.stop();
            }
            last_aimpoints = std::move(aimpoints);

            const auto indices = last_aimpoints.valid_indices();
            /*^^*/ if (indices.size() == 1) {
                if (single_rune_actions.started() == false) {
                    single_rune_actions.start();
                }

                const auto tag = single_rune_actions.tick();
                if (tag == "idle") {
                    return { static_cast<std::int8_t>(indices[0]), true };
                } else {
                    return { static_cast<std::int8_t>(indices[0]), false };
                }
            } else if (indices.size() == 2) {
                if (multiple_rune_actions.started() == false) {
                    multiple_rune_actions.start();
                }

                const auto tag = multiple_rune_actions.tick();
                /*^^*/ if (tag == "1-idle") {
                    return { static_cast<std::int8_t>(indices[0]), true };
                } else if (tag == "1-shoot") {
                    return { static_cast<std::int8_t>(indices[0]), false };
                } else if (tag == "2-idle") {
                    return { static_cast<std::int8_t>(indices[1]), true };
                } else if (tag == "2-shoot") {
                    return { static_cast<std::int8_t>(indices[1]), false };
                }
            }

            return { -1, true };
        }

        // 目标身份变化时重置滞回状态
        if (use_pre_aim && trackable.id() != last_device) {
            last_device = trackable.id();
            last_idx    = -1;
        }

        // 窗口计算
        auto degraded    = false;
        auto half_window = double { };

        if (use_pre_aim) {
            auto dynamic_window = std::numeric_limits<double>::infinity();
            if (aimpoints.size() == 4) {
                std::tie(dynamic_window, degraded) = get_attack_window(trackable);
            }
            half_window = std::min(config.attack_window / 2, dynamic_window);
        } else {
            half_window = config.attack_window / 2;
        }

        // 退化：直接返回哨兵值，由 aim() 瞄准中心
        if (degraded) {
            last_idx = -1;
            return { std::numeric_limits<std::int8_t>::max(), false };
        }

        // 施密特滞回：进入阈值收紧，退出阈值放宽；raw 评估保持无状态
        const auto enter_window =
            use_pre_aim ? half_window * (1.0 - config.window_hysteresis) : half_window;
        const auto exit_window = half_window * (1.0 + config.window_hysteresis);

        // 装甲板选择
        {
            const auto length = static_cast<std::int8_t>(aimpoints.size());
            if (use_pre_aim && last_idx >= 0 && last_idx < length) {
                const auto delta = util::normalize_angle(
                    yaw_angle(trackable.get_direction(), aimpoints[last_idx]));
                if (std::abs(delta) <= exit_window) {
                    return { last_idx, false };
                }
            }

            auto best_index = std::optional<std::int8_t> { };
            auto best_delta = std::numeric_limits<double>::max();
            auto next_index = std::optional<std::int8_t> { };
            auto next_delta = std::numeric_limits<double>::max();

            for (auto i = std::size_t { 0 }; i < aimpoints.size(); ++i) {
                const auto delta =
                    util::normalize_angle(yaw_angle(trackable.get_direction(), aimpoints[i]));
                const auto abs_delta = std::abs(delta);

                if (abs_delta <= enter_window && abs_delta < best_delta) {
                    best_delta = abs_delta;
                    best_index = static_cast<std::int8_t>(i);
                }

                if (use_pre_aim) {
                    const auto dominated = (omega < 0) ? (delta > 0) : (delta < 0);
                    if (dominated && abs_delta < next_delta) {
                        next_delta = abs_delta;
                        next_index = static_cast<std::int8_t>(i);
                    }
                }
            }

            if (best_index) {
                if (use_pre_aim) last_idx = *best_index;
                return { *best_index, false };
            }
            if (next_index && use_pre_aim) {
                last_idx = *next_index;
                return { *next_index, true };
            }
            if (use_pre_aim) last_idx = -1;
            return { -1, false };
        }
    }

    auto aim(const Trackable& trackable) -> std::optional<Aimed> {

        const auto direction = trackable.get_direction();
        const auto distance  = std::sqrt(
            direction.x * direction.x + direction.y * direction.y + direction.z * direction.z);

        const auto fly_time  = distance / config.bullet_speed;
        const auto increment = config.shoot_delay
            + std::chrono::duration<double>(state.timestamp - trackable.get_timestamp()).count();

        auto aim_selected = std::int8_t { -1 };
        auto raw_selected = std::int8_t { -1 };
        auto pre_aim      = false;

        {
            auto future = trackable.clone();
            future->jump_into(fly_time + increment);
            std::tie(aim_selected, pre_aim)     = evaluate_aimed(*future);
            std::tie(raw_selected, std::ignore) = evaluate_aimed(*future, false);
        }
        if (aim_selected < 0) return std::nullopt;

        const auto degraded = (aim_selected == std::numeric_limits<std::int8_t>::max());

        constexpr auto kMaxIterate = std::size_t { 5 };
        constexpr auto kEpsilon    = 0.001;

        auto attack = Point3d { };
        auto center = Point3d { };
        auto yaw    = double { };
        auto pitch  = double { };

        auto new_time = double { fly_time };

        auto solution = TrajectorySolution { };
        for (auto i = std::size_t { 0 }; i < kMaxIterate; ++i) {
            auto clone = trackable.clone();
            clone->jump_into(new_time + increment);

            center = clone->get_direction();
            attack = degraded ? center : clone->get_aimpoints()[aim_selected];

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

        if (degraded) {
            return Aimed {
                .aim_yaw = yaw,
                .raw_yaw = yaw,
                .pitch   = pitch,
                .shoot   = true,
                .pre_aim = false,
                .target  = make_target(yaw, pitch),
                .center  = center,
                .attack  = center,
            };
        }

        // 原始 yaw 计算
        auto raw_yaw = double { yaw };
        if (raw_selected >= 0 && raw_selected != aim_selected) {
            auto raw_clone = trackable.clone();
            raw_clone->jump_into(new_time + increment);

            const auto raw_aimpoints = raw_clone->get_aimpoints();
            const auto raw_attack    = raw_aimpoints[raw_selected];

            TrajectorySolution solution { };
            solution.input.v0    = config.bullet_speed;
            solution.input.point = raw_attack;

            const auto result = solution.solve();
            if (!result) return std::nullopt;

            raw_yaw = util::normalize_angle(result->yaw + config.offset_yaw);
        }

        // 射击评估
        auto should_shoot = true;
        if (pre_aim && !config.attack_preaim) {
            should_shoot = false;
        } else {
            const auto cmd = ShootEvaluator::Command {
                .yaw    = yaw,
                .pitch  = pitch,
                .center = center,
                .armor  = attack,
            };
            should_shoot = shoot_evaluator->evaluate(cmd, state.yaw, state.pitch);
        }

        return Aimed {
            .aim_yaw = yaw,
            .raw_yaw = raw_yaw,
            .pitch   = pitch,
            .shoot   = should_shoot,
            .pre_aim = pre_aim,
            .target  = make_target(yaw, pitch),
            .center  = center,
            .attack  = attack,
        };
    }
};

FireController::FireController(const YAML::Node& yaml)
    : pimpl { std::make_unique<Impl>(yaml) } { }

FireController::FireController(const Config& config)
    : pimpl { std::make_unique<Impl>(config) } { }

FireController::~FireController() noexcept = default;

auto FireController::update(State state) -> void { pimpl->state = state; }

auto FireController::aim(const Trackable& trackable) -> std::optional<Aimed> {
    return pimpl->aim(trackable);
}
