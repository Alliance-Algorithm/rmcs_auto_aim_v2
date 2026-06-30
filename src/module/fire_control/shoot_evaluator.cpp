#include "shoot_evaluator.hpp"

#include <cmath>
#include <optional>

#include "utility/math/angle.hpp"
#include "utility/serializable.hpp"

using namespace rmcs::fire_control;

struct ShootEvaluator::Impl {
    struct Config : util::Serializable {
        double near_yaw_tolerance { 3.0 };
        double far_yaw_tolerance { 2.0 };
        double near_pitch_tolerance { 2.0 };
        double far_pitch_tolerance { 1.0 };
        double split_distance { 3.0 };
        bool is_lazy_gimbal { false };

        constexpr static std::tuple metas {
            &Config::near_yaw_tolerance,
            "near_yaw_tolerance",
            &Config::far_yaw_tolerance,
            "far_yaw_tolerance",
            &Config::near_pitch_tolerance,
            "near_pitch_tolerance",
            &Config::far_pitch_tolerance,
            "far_pitch_tolerance",
            &Config::split_distance,
            "split_distance",
            &Config::is_lazy_gimbal,
            "is_lazy_gimbal",
        };
    } config;

    auto configure_yaml(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
        auto result = config.serialize(yaml);
        if (!result.has_value()) {
            return std::unexpected { result.error() };
        }

        config.near_yaw_tolerance   = util::deg2rad(config.near_yaw_tolerance);
        config.far_yaw_tolerance    = util::deg2rad(config.far_yaw_tolerance);
        config.near_pitch_tolerance = util::deg2rad(config.near_pitch_tolerance);
        config.far_pitch_tolerance  = util::deg2rad(config.far_pitch_tolerance);
        last_command_.reset();

        if (!(config.near_yaw_tolerance > 0.0) || !(config.far_yaw_tolerance > 0.0)) {
            return std::unexpected {
                "near_yaw_tolerance and far_yaw_tolerance must be > 0",
            };
        }

        if (!(config.near_pitch_tolerance > 0.0) || !(config.far_pitch_tolerance > 0.0)) {
            return std::unexpected {
                "near_pitch_tolerance and far_pitch_tolerance must be > 0",
            };
        }

        if (config.split_distance < 0.0) {
            return std::unexpected { "split_distance must be >= 0" };
        }

        return { };
    }

    auto evaluate(Command const& command, GimbalState const& state) noexcept -> bool {
        auto should_fire = false;

        if (config.is_lazy_gimbal) {
            auto const aim_point_yaw = std::atan2(command.attack.y(), command.attack.x());
            const auto aim_delta     = std::abs(util::normalize_angle(state.yaw - aim_point_yaw));
            const auto is_overlap    = aim_delta < config.near_yaw_tolerance;
            if (!is_overlap) {
                last_command_ = command;
                return false;
            }
        }

        auto const center_distance = std::hypot(command.center.x(), command.center.y());
        const auto yaw_tolerance   = (center_distance > config.split_distance)
            ? config.far_yaw_tolerance
            : config.near_yaw_tolerance;
        const auto pitch_tolerance = (center_distance > config.split_distance)
            ? config.far_pitch_tolerance
            : config.near_pitch_tolerance;

        if (last_command_.has_value()) {
            const auto yaw_delta =
                std::abs(util::normalize_angle(last_command_->yaw - command.yaw));
            const auto track_delta =
                std::abs(util::normalize_angle(state.yaw - last_command_->yaw));
            const auto pitch_delta = std::abs(util::normalize_angle(state.pitch - command.pitch));

            should_fire = (yaw_delta < yaw_tolerance) && (track_delta < yaw_tolerance)
                && (pitch_delta < pitch_tolerance);
        }

        last_command_ = command;
        return should_fire;
    }

private:
    std::optional<Command> last_command_ { };
};

ShootEvaluator::ShootEvaluator() noexcept
    : pimpl { std::make_unique<Impl>() } { }

ShootEvaluator::~ShootEvaluator() noexcept = default;

auto ShootEvaluator::initialize(const YAML::Node& yaml) noexcept
    -> std::expected<void, std::string> {
    return pimpl->configure_yaml(yaml);
}

auto ShootEvaluator::configure_yaml(const YAML::Node& yaml) noexcept
    -> std::expected<void, std::string> {
    return pimpl->configure_yaml(yaml);
}

auto ShootEvaluator::evaluate(Command const& command, GimbalState const& state) noexcept -> bool {
    return pimpl->evaluate(command, state);
}
