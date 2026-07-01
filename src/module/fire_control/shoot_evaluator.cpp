#include "shoot_evaluator.hpp"

#include <cmath>
#include <optional>

#include "utility/logging/printer.hpp"
#include "utility/math/angle.hpp"

using namespace rmcs::fire_control;

struct ShootEvaluator::Impl {
    static constexpr auto kMinDistance = 1e-6;

    Config config;

    struct YawWindow {
        double right;
        double left;
    };

    struct PitchWindow {
        double lower;
        double upper;
    };

    static auto finite(double value) noexcept -> bool { return std::isfinite(value); }

    static auto finite(const Eigen::Vector3d& value) noexcept -> bool { return value.allFinite(); }

    static auto finite(Command const& command) noexcept -> bool {
        return finite(command.yaw) && finite(command.pitch) && finite(command.center)
            && finite(command.attack);
    }

    static auto finite(double yaw, double pitch) noexcept -> bool {
        return finite(yaw) && finite(pitch);
    }

    static auto in_yaw_window(double yaw, double center, const YawWindow& window) noexcept -> bool {
        const auto error = util::normalize_angle(yaw - center);
        return error >= window.right && error <= window.left;
    }

    static auto in_pitch_window(double pitch, const PitchWindow& window) noexcept -> bool {
        return pitch >= window.lower && pitch <= window.upper;
    }

    static auto make_logging(double xy, YawWindow yaw, PitchWindow pit) {
        static auto temp = Printer { "ShootEvaluator" };
        temp.info("distance: {:.2f}, yaw: [{:.2f}, {:.2f}], pitch: [{:.2f}, {:.2f}]", xy, yaw.left,
            yaw.right, pit.upper, pit.lower);
    }

    explicit Impl(const Config& config)
        : config { config } { }

    auto yaw_window(Command const& command) const noexcept -> YawWindow {
        const auto yaw_center = std::atan2(command.attack.y(), command.attack.x());
        const auto lateral    = Eigen::Vector3d {
            -std::sin(yaw_center),
            +std::cos(yaw_center),
            0.0,
        };

        const auto left_point  = command.attack + lateral * config.yaw_tolerance;
        const auto right_point = command.attack - lateral * config.yaw_tolerance;

        return {
            .right =
                util::normalize_angle(std::atan2(right_point.y(), right_point.x()) - yaw_center),
            .left = util::normalize_angle(std::atan2(left_point.y(), left_point.x()) - yaw_center),
        };
    }

    auto pitch_window(Command const& command) const noexcept -> PitchWindow {
        const auto distance_xy  = std::hypot(command.attack.x(), command.attack.y());
        const auto pitch_center = std::atan2(command.attack.z(), distance_xy);

        const auto pitch_lower =
            std::atan2(command.attack.z() - config.pitch_tolerance, distance_xy);
        const auto pitch_upper =
            std::atan2(command.attack.z() + config.pitch_tolerance, distance_xy);

        return {
            .lower = command.pitch + (pitch_lower - pitch_center),
            .upper = command.pitch + (pitch_upper - pitch_center),
        };
    }

    auto evaluate(Command const& command, double yaw, double pitch) noexcept -> bool {
        if (!finite(command) || !finite(yaw, pitch)) {
            last_command.reset();
            return false;
        }

        const auto distance_xy = std::hypot(command.attack.x(), command.attack.y());
        if (!(distance_xy > kMinDistance)) {
            last_command.reset();
            return false;
        }

        const auto yaw_window   = this->yaw_window(command);
        const auto pitch_window = this->pitch_window(command);

        if (config.is_lazy_gimbal) {
            const auto aim_point_yaw = std::atan2(command.attack.y(), command.attack.x());
            if (!in_yaw_window(yaw, aim_point_yaw, yaw_window)) {
                last_command = command;
                return false;
            }
        }

        const auto aim_aligned =
            in_yaw_window(yaw, command.yaw, yaw_window) && in_pitch_window(pitch, pitch_window);

        auto command_stable = false;
        if (last_command.has_value()) {
            command_stable = in_yaw_window(last_command->yaw, command.yaw, yaw_window)
                && in_pitch_window(last_command->pitch, pitch_window);
        }

        last_command = command;
        return aim_aligned && (!config.require_stable_command || command_stable);
    }

private:
    std::optional<Command> last_command { };
};

ShootEvaluator::ShootEvaluator(const Config& config)
    : pimpl { std::make_unique<Impl>(config) } { }

ShootEvaluator::~ShootEvaluator() noexcept = default;

auto ShootEvaluator::evaluate(Command const& command, double yaw, double pitch) noexcept -> bool {
    return pimpl->evaluate(command, yaw, pitch);
}
