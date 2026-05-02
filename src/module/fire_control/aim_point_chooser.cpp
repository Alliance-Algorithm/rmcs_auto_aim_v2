#include "aim_point_chooser.hpp"

#include <cmath>
#include <tuple>
#include <vector>

#include "utility/math/conversion.hpp"
#include "utility/serializable.hpp"

using namespace rmcs::fire_control;

struct AimPointChooser::Impl {
    struct Config : util::Serializable {
        double coming_angle { 60.0 };
        double leaving_angle { 20.0 };
        double outpost_coming_angle { 70.0 };
        double outpost_leaving_angle { 30.0 };

        constexpr static std::tuple metas {
            &Config::coming_angle,
            "coming_angle",
            &Config::leaving_angle,
            "leaving_angle",
            &Config::outpost_coming_angle,
            "outpost_coming_angle",
            &Config::outpost_leaving_angle,
            "outpost_leaving_angle",
        };
    } config;

    struct CandidateEval {
        double delta_yaw { 0.0 };
        bool in_window { false };
    };

    const double min_switch_improvement_angle { util::deg2rad(8.0) };
    std::optional<int> last_chosen_armor_id {};

    auto initialize(AimPointChooser::Config const& external_config) noexcept -> void {
        config.coming_angle          = external_config.coming_angle;
        config.leaving_angle         = external_config.leaving_angle;
        config.outpost_coming_angle  = external_config.outpost_coming_angle;
        config.outpost_leaving_angle = external_config.outpost_leaving_angle;
        last_chosen_armor_id.reset();
    }

    auto configure_yaml(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
        auto result = config.serialize(yaml);
        if (!result.has_value()) return std::unexpected { result.error() };

        config.coming_angle          = util::deg2rad(config.coming_angle);
        config.leaving_angle         = util::deg2rad(config.leaving_angle);
        config.outpost_coming_angle  = util::deg2rad(config.outpost_coming_angle);
        config.outpost_leaving_angle = util::deg2rad(config.outpost_leaving_angle);
        last_chosen_armor_id.reset();

        if (!(config.coming_angle > 0.0) || !(config.leaving_angle > 0.0)
            || !(config.outpost_coming_angle > 0.0) || !(config.outpost_leaving_angle > 0.0)) {
            return std::unexpected {
                "coming_angle, leaving_angle, outpost_coming_angle and outpost_leaving_angle must be > 0",
            };
        }

        return {};
    }

    auto choose_armor(std::span<Armor3D const> armors, Eigen::Vector3d const& center_position,
        double angular_velocity) -> std::optional<Armor3D> {
        if (armors.empty()) {
            last_chosen_armor_id.reset();
            return std::nullopt;
        }

        const auto center_yaw    = std::atan2(center_position.y(), center_position.x());
        const auto is_outpost    = armors.front().genre == DeviceId::OUTPOST;
        const auto active_coming = is_outpost ? config.outpost_coming_angle : config.coming_angle;
        const auto active_leaving =
            is_outpost ? config.outpost_leaving_angle : config.leaving_angle;

        auto candidate_evals = std::vector<CandidateEval>(armors.size());

        const auto yaw = [&](size_t index) {
            auto orientation = Eigen::Quaterniond {};
            armors[index].orientation.copy_to(orientation);
            return util::eulers(orientation)[0];
        };

        const auto in_window = [&](double delta_yaw) {
            const auto abs_delta  = std::abs(delta_yaw);
            const auto in_coming  = abs_delta <= active_coming;
            const auto in_leaving = (angular_velocity > 0.0) ? (delta_yaw <= active_leaving)
                : (angular_velocity < 0.0)                   ? (delta_yaw >= -active_leaving)
                                                             : true;
            return in_coming && in_leaving;
        };

        for (size_t index = 0; index < armors.size(); ++index) {
            const auto delta_yaw  = util::normalize_angle(yaw(index) - center_yaw);
            candidate_evals[index] = {
                .delta_yaw = delta_yaw,
                .in_window = in_window(delta_yaw),
            };
        }

        const auto priority_key = [&](size_t index) {
            const auto abs_delta    = std::abs(candidate_evals[index].delta_yaw);
            const auto id           = armors[index].id;
            const auto is_last      = last_chosen_armor_id.has_value() && (id == *last_chosen_armor_id);
            const auto last_penalty = is_last ? 0 : 1;
            return std::tuple { abs_delta, last_penalty, id, index };
        };

        auto best_idx = std::optional<size_t> {};
        auto last_idx = std::optional<size_t> {};

        for (size_t index = 0; index < armors.size(); ++index) {
            if (last_chosen_armor_id.has_value() && (armors[index].id == *last_chosen_armor_id)) {
                last_idx = index;
            }

            if (!candidate_evals[index].in_window) continue;

            if (!best_idx.has_value() || (priority_key(index) < priority_key(*best_idx))) {
                best_idx = index;
            }
        }

        if (!best_idx.has_value()) {
            last_chosen_armor_id.reset();
            return std::nullopt;
        }

        if (last_idx.has_value() && (*last_idx != *best_idx)) {
            const auto last_abs    = std::abs(candidate_evals[*last_idx].delta_yaw);
            const auto best_abs    = std::abs(candidate_evals[*best_idx].delta_yaw);
            const auto improvement = last_abs - best_abs;
            if (improvement < min_switch_improvement_angle) {
                best_idx = last_idx;
            }
        }

        last_chosen_armor_id = armors[*best_idx].id;
        return { armors[*best_idx] };
    }
};

AimPointChooser::AimPointChooser() noexcept
    : pimpl { std::make_unique<Impl>() } { }

AimPointChooser::~AimPointChooser() noexcept = default;

auto AimPointChooser::initialize(Config const& config) noexcept -> void {
    pimpl->initialize(config);
}

auto AimPointChooser::configure_yaml(const YAML::Node& yaml) noexcept
    -> std::expected<void, std::string> {
    return pimpl->configure_yaml(yaml);
}

auto AimPointChooser::choose_armor(std::span<Armor3D const> armors,
    Eigen::Vector3d const& center_position, double angular_velocity) -> std::optional<Armor3D> {
    return pimpl->choose_armor(armors, center_position, angular_velocity);
}
