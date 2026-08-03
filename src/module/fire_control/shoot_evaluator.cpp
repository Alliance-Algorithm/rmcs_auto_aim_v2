#include "shoot_evaluator.hpp"
#include "utility/math/angle.hpp"

using namespace rmcs;

struct ShootEvaluator::Impl {
    // 角容差距离上限：超过该距离时窗口角宽保持此处的等效值，不再随距离收窄
    static constexpr auto kMinDistance = 0.01;
    static constexpr auto kMaxDistance = 5.00;

    Config config;

    struct YawWindow {
        double right;
        double left;
    };

    struct PitchWindow {
        double lower;
        double upper;
    };

    static constexpr auto finite(double value) noexcept -> bool { return std::isfinite(value); }

    static constexpr auto finite(const Point3d& p) noexcept -> bool {
        return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
    }

    static constexpr auto finite(Command const& command) noexcept -> bool {
        return finite(command.yaw) && finite(command.pitch) //
            && finite(command.center) && finite(command.armor);
    }

    static constexpr auto finite(double yaw, double pitch) noexcept -> bool {
        return finite(yaw) && finite(pitch);
    }

    static constexpr auto yaw_angle(const Point3d& center, const Point3d& armor) -> double {
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

    static auto in_yaw_window(double yaw, double center, const YawWindow& window) noexcept -> bool {
        const auto error = util::normalize_angle(yaw - center);
        return error >= window.right && error <= window.left;
    }

    static constexpr auto in_pitch_window(
        double pitch, double center, const PitchWindow& window) noexcept -> bool {
        const auto error = pitch - center;
        return error >= window.lower && error <= window.upper;
    }

    static constexpr auto yaw_window(const Point3d& attack, double yaw_tolerance) noexcept
        -> YawWindow {
        const auto yaw_center = std::atan2(attack.y, attack.x);
        const auto lat_x      = -std::sin(yaw_center);
        const auto lat_y      = +std::cos(yaw_center);

        const auto left_x  = attack.x + lat_x * yaw_tolerance;
        const auto left_y  = attack.y + lat_y * yaw_tolerance;
        const auto right_x = attack.x - lat_x * yaw_tolerance;
        const auto right_y = attack.y - lat_y * yaw_tolerance;

        return {
            .right = util::normalize_angle(std::atan2(right_y, right_x) - yaw_center),
            .left  = util::normalize_angle(std::atan2(left_y, left_x) - yaw_center),
        };
    }

    static constexpr auto pitch_window(const Point3d& attack, double pitch_tolerance) noexcept
        -> PitchWindow {
        const auto distance_xy  = std::hypot(attack.x, attack.y);
        const auto pitch_center = std::atan2(attack.z, distance_xy);

        const auto pitch_lower = std::atan2(attack.z - pitch_tolerance, distance_xy);
        const auto pitch_upper = std::atan2(attack.z + pitch_tolerance, distance_xy);

        return {
            .lower = pitch_lower - pitch_center,
            .upper = pitch_upper - pitch_center,
        };
    }

    explicit Impl(const Config& config)
        : config { config } { }

    auto evaluate(Command const& command, double yaw, double pitch) noexcept {
        if (!finite(command) || !finite(yaw, pitch)) {
            return false;
        }

        const auto distance_xy = std::hypot(command.armor.x, command.armor.y);
        if (distance_xy < kMinDistance) {
            return false;
        }

        const auto ratio  = std::min(distance_xy, kMaxDistance) / distance_xy;
        const auto attack = Point3d {
            command.armor.x * ratio,
            command.armor.y * ratio,
            command.armor.z * ratio,
        };

        const auto scale = std::cos(yaw_angle(command.center, command.armor));

        const auto scaled_yaw_tol   = config.yaw_tolerance * scale;
        const auto scaled_pitch_tol = config.pitch_tolerance * scale;

        const auto yaw_win   = this->yaw_window(attack, scaled_yaw_tol);
        const auto pitch_win = this->pitch_window(attack, scaled_pitch_tol);

        return in_yaw_window(yaw, command.yaw, yaw_win)
            && in_pitch_window(pitch, command.pitch, pitch_win);
    }
};

ShootEvaluator::ShootEvaluator(const Config& config)
    : pimpl { std::make_unique<Impl>(config) } { }

ShootEvaluator::~ShootEvaluator() noexcept = default;

auto ShootEvaluator::evaluate(Command const& command, double yaw, double pitch) noexcept -> bool {
    return pimpl->evaluate(command, yaw, pitch);
}
