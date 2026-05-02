#include "shoot_evaluator.hpp"

#include <cmath>
#include <optional>

#include "utility/math/angle.hpp"
#include "utility/serializable.hpp"

using namespace rmcs::fire_control;

struct ShootEvaluator::Impl {
    struct Config : util::Serializable {
        double near_angle_tolerance { 4.0 };
        double far_angle_tolerance { 2.0 };
        double split_distance { 3.0 };
        bool auto_fire { true };
        bool is_lazy_gimbal { false };

        constexpr static std::tuple metas {
            &Config::near_angle_tolerance,
            "near_angle_tolerance",
            &Config::far_angle_tolerance,
            "far_angle_tolerance",
            &Config::split_distance,
            "split_distance",
            &Config::auto_fire,
            "auto_fire",
            &Config::is_lazy_gimbal,
            "is_lazy_gimbal",
        };
    } config;

    auto configure_yaml(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
        auto result = config.serialize(yaml);
        if (!result.has_value()) {
            return std::unexpected { result.error() };
        }

        config.near_angle_tolerance = util::deg2rad(config.near_angle_tolerance);
        config.far_angle_tolerance  = util::deg2rad(config.far_angle_tolerance);
        last_command_.reset();

        if (!(config.near_angle_tolerance > 0.0) || !(config.far_angle_tolerance > 0.0)) {
            return std::unexpected {
                "near_angle_tolerance and far_angle_tolerance must be > 0",
            };
        }

        if (config.split_distance < 0.0) {
            return std::unexpected { "split_distance must be >= 0" };
        }

        return {};
    }

    auto evaluate(Command const& command, double current_yaw) noexcept -> bool {
        auto should_fire = false;

        if (!command.control || !config.auto_fire) {
            last_command_ = command;
            return false;
        }

        if (config.is_lazy_gimbal) {
            auto const aim_point_yaw =
                std::atan2(command.aim_point_position.y(), command.aim_point_position.x());
            const auto aim_delta  = std::abs(util::normalize_angle(current_yaw - aim_point_yaw));
            const auto is_overlap = aim_delta < config.near_angle_tolerance;
            if (!is_overlap) {
                last_command_ = command;
                return false;
            }
        }

        auto const center_distance =
            std::hypot(command.center_position.x(), command.center_position.y());
        const auto tolerance = (center_distance > config.split_distance)
            ? config.far_angle_tolerance
            : config.near_angle_tolerance;

        if (last_command_.has_value() && command.auto_aim_enabled) {
            const auto yaw_delta =
                std::abs(util::normalize_angle(last_command_->yaw - command.yaw));
            const auto track_delta =
                std::abs(util::normalize_angle(current_yaw - last_command_->yaw));

            should_fire = (yaw_delta < tolerance * 2.0) && (track_delta < tolerance);
        }

        last_command_ = command;
        return should_fire;
    }

private:
    std::optional<Command> last_command_ {};
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

auto ShootEvaluator::evaluate(Command const& command, double current_yaw) noexcept -> bool {
    return pimpl->evaluate(command, current_yaw);
}
