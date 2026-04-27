#include "shoot_evaluator.hpp"

#include <cmath>
#include <optional>

#include "utility/math/angle.hpp"

using namespace rmcs::fire_control;

struct ShootEvaluator::Impl {

    auto initialize(Config const& config_in) noexcept -> std::expected<void, std::string> {
        config = config_in;
        last_command_.reset();

        if (!(config.first_tolerance > 0.0) || !(config.second_tolerance > 0.0))
            return std::unexpected { "first_tolerance and second_tolerance must be > 0" };

        if (config.judge_distance < 0.0) return std::unexpected { "judge_distance must be >= 0" };

        return {};
    }

    auto evaluate(Command const& command, double current_yaw) noexcept -> bool {
        auto should_fire = false;

        if (!command.control || !config.auto_fire) {
            last_command_ = command;
            return false;
        }

        const auto tolerance = (command.distance > config.judge_distance) ? config.second_tolerance
                                                                          : config.first_tolerance;

        if (last_command_.has_value() && command.auto_aim_enabled && command.aim_point_valid) {
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
    Config config {};

    std::optional<Command> last_command_ {};
};

ShootEvaluator::ShootEvaluator() noexcept
    : pimpl { std::make_unique<Impl>() } { }

ShootEvaluator::~ShootEvaluator() noexcept = default;

auto ShootEvaluator::initialize(Config const& config) noexcept -> std::expected<void, std::string> {
    return pimpl->initialize(config);
}

auto ShootEvaluator::evaluate(Command const& command, double current_yaw) noexcept -> bool {
    return pimpl->evaluate(command, current_yaw);
}
