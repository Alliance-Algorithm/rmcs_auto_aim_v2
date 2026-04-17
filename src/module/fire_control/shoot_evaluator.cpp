#include "shoot_evaluator.hpp"

#include <cmath>
#include <optional>

#include "utility/math/angle.hpp"
#include "utility/serializable.hpp"

using namespace rmcs::fire_control;

struct ShootEvaluator::Impl {
    struct Config : util::Serializable {
        double first_tolerance { 4.0 };  // degree
        double second_tolerance { 2.0 }; // degree
        double judge_distance { 3.0 };   // m
        bool auto_fire { true };

        constexpr static std::tuple metas {
            &Config::first_tolerance,
            "first_tolerance",
            &Config::second_tolerance,
            "second_tolerance",
            &Config::judge_distance,
            "judge_distance",
            &Config::auto_fire,
            "auto_fire",
        };
    };

    Config config {};

    double first_tolerance_ { 4.0 / 57.3 };
    double second_tolerance_ { 2.0 / 57.3 };
    double judge_distance_ { 3.0 };
    bool auto_fire_ { true };

    std::optional<Command> last_command_ {};

    auto initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
        auto result = config.serialize(yaml);
        if (!result.has_value()) {
            return std::unexpected { result.error() };
        }

        first_tolerance_  = config.first_tolerance / 57.3;
        second_tolerance_ = config.second_tolerance / 57.3;
        judge_distance_   = config.judge_distance;
        auto_fire_        = config.auto_fire;
        last_command_.reset();

        if (!(first_tolerance_ > 0.0) || !(second_tolerance_ > 0.0)) {
            return std::unexpected { "first_tolerance and second_tolerance must be > 0" };
        }
        if (judge_distance_ < 0.0) {
            return std::unexpected { "judge_distance must be >= 0" };
        }

        return {};
    }

    auto evaluate(Command const& command, double current_yaw) noexcept -> bool {
        auto should_fire = false;

        if (!command.control || !auto_fire_) {
            last_command_ = command;
            return false;
        }

        const auto tolerance =
            (command.distance > judge_distance_) ? second_tolerance_ : first_tolerance_;

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
};

ShootEvaluator::ShootEvaluator() noexcept
    : pimpl { std::make_unique<Impl>() } { }

ShootEvaluator::~ShootEvaluator() noexcept = default;

auto ShootEvaluator::initialize(const YAML::Node& yaml) noexcept
    -> std::expected<void, std::string> {
    return pimpl->initialize(yaml);
}

auto ShootEvaluator::evaluate(Command const& command, double current_yaw) noexcept -> bool {
    return pimpl->evaluate(command, current_yaw);
}
