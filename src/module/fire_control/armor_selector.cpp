#include "armor_selector.hpp"

#include <cmath>
#include <tuple>
#include <vector>

#include <eigen3/Eigen/Geometry>

#include "utility/math/angle.hpp"
#include "utility/math/conversion.hpp"
#include "utility/serializable.hpp"

using namespace rmcs::fire_control;

struct ArmorSelector::Impl {
    struct Config : util::Serializable {
        double coming_angle { 60.0 };
        double leaving_angle { 20.0 };
        double outpost_coming_angle { 70.0 };
        double outpost_leaving_angle { 30.0 };
        double switch_threshold { 8.0 };

        constexpr static std::tuple metas {
            &Config::coming_angle,
            "coming_angle",
            &Config::leaving_angle,
            "leaving_angle",
            &Config::outpost_coming_angle,
            "outpost_coming_angle",
            &Config::outpost_leaving_angle,
            "outpost_leaving_angle",
            &Config::switch_threshold,
            "switch_threshold",
        };
    } config;

    struct CandidateEval {
        double delta_yaw { 0.0 };
        bool in_window { false };
    };

    auto configure_yaml(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
        auto result = config.serialize(yaml);
        if (!result.has_value()) return std::unexpected { result.error() };

        config.coming_angle          = util::deg2rad(config.coming_angle);
        config.leaving_angle         = util::deg2rad(config.leaving_angle);
        config.outpost_coming_angle  = util::deg2rad(config.outpost_coming_angle);
        config.outpost_leaving_angle = util::deg2rad(config.outpost_leaving_angle);
        config.switch_threshold      = util::deg2rad(config.switch_threshold);

        if (!(config.coming_angle > 0.0) || !(config.leaving_angle > 0.0)
            || !(config.outpost_coming_angle > 0.0) || !(config.outpost_leaving_angle > 0.0)
            || !(config.switch_threshold > 0.0)) {
            return std::unexpected {
                "coming_angle, leaving_angle, outpost_coming_angle, outpost_leaving_angle and "
                "switch_threshold must be > 0",
            };
        }

        return {};
    }

    auto select(std::span<ArmorCandidate const> candidates,
        std::optional<int> last_selected_armor_id) const -> std::optional<size_t> {
        if (candidates.empty()) return std::nullopt;

        const auto is_outpost    = candidates.front().armor.genre == DeviceId::OUTPOST;
        const auto active_coming = is_outpost ? config.outpost_coming_angle : config.coming_angle;
        const auto active_leaving =
            is_outpost ? config.outpost_leaving_angle : config.leaving_angle;

        auto candidate_evals = std::vector<CandidateEval>(candidates.size());

        const auto in_window = [&](double delta_yaw, double angular_velocity) {
            const auto abs_delta  = std::abs(delta_yaw);
            const auto in_coming  = abs_delta <= active_coming;
            const auto in_leaving = (angular_velocity > 0.0) ? (delta_yaw <= active_leaving)
                : (angular_velocity < 0.0)                   ? (delta_yaw >= -active_leaving)
                                                             : true;
            return in_coming && in_leaving;
        };

        const auto delta_yaw = [](ArmorCandidate const& candidate) {
            auto orientation = Eigen::Quaterniond {};
            candidate.armor.orientation.copy_to(orientation);
            const auto armor_yaw  = util::eulers(orientation)[0];
            const auto& center    = candidate.motion.center_position;
            const auto center_yaw = std::atan2(center.y, center.x);
            return util::normalize_angle(armor_yaw - center_yaw);
        };

        for (size_t index = 0; index < candidates.size(); ++index) {
            const auto delta       = delta_yaw(candidates[index]);
            candidate_evals[index] = {
                .delta_yaw = delta,
                .in_window = in_window(delta, candidates[index].motion.angular_velocity),
            };
        }

        const auto priority_key = [&](size_t index) {
            const auto leaving_penalty = [&] {
                const auto delta = candidate_evals[index].delta_yaw;
                if (candidates[index].motion.angular_velocity > 0.0) return (delta > 0.0) ? 1 : 0;
                if (candidates[index].motion.angular_velocity < 0.0) return (delta < 0.0) ? 1 : 0;
                return 0;
            }();
            const auto abs_delta = std::abs(candidate_evals[index].delta_yaw);
            const auto id        = candidates[index].armor.id;
            const auto is_last =
                last_selected_armor_id.has_value() && (id == *last_selected_armor_id);
            const auto last_penalty = is_last ? 0 : 1;
            return std::tuple { leaving_penalty, abs_delta, last_penalty, id, index };
        };

        auto best_idx = std::optional<size_t> {};
        auto last_idx = std::optional<size_t> {};

        for (size_t index = 0; index < candidates.size(); ++index) {
            if (!candidate_evals[index].in_window) continue;

            if (last_selected_armor_id.has_value()
                && (candidates[index].armor.id == *last_selected_armor_id)) {
                last_idx = index;
            }

            if (!best_idx.has_value() || (priority_key(index) < priority_key(*best_idx))) {
                best_idx = index;
            }
        }

        if (!best_idx.has_value()) return std::nullopt;

        if (last_idx.has_value() && (*last_idx != *best_idx)) {
            const auto last_abs    = std::abs(candidate_evals[*last_idx].delta_yaw);
            const auto best_abs    = std::abs(candidate_evals[*best_idx].delta_yaw);
            const auto improvement = last_abs - best_abs;
            if (improvement < config.switch_threshold) {
                best_idx = last_idx;
            }
        }

        return best_idx;
    }
};

ArmorSelector::ArmorSelector() noexcept
    : pimpl { std::make_unique<Impl>() } { }

ArmorSelector::~ArmorSelector() noexcept = default;

auto ArmorSelector::configure_yaml(const YAML::Node& yaml) noexcept
    -> std::expected<void, std::string> {
    return pimpl->configure_yaml(yaml);
}

auto ArmorSelector::select(std::span<ArmorCandidate const> candidates,
    std::optional<int> last_selected_armor_id) const -> std::optional<size_t> {
    return pimpl->select(candidates, last_selected_armor_id);
}
