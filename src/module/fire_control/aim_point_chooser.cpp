#include "aim_point_chooser.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include "utility/math/conversion.hpp"

using namespace rmcs::fire_control;

struct AimPointChooser::Impl {
    static constexpr int kNoArmor = -1;

    struct ArmorCandidate {
        int index { kNoArmor };
        double delta_yaw { 0.0 };
        double phase { 0.0 };
    };

    struct AngleWindow {
        double coming { 0.0 };
        double leaving { 0.0 };
    };

    AngleWindow normal_window { util::deg2rad(60.0), util::deg2rad(20.0) };  // rad
    AngleWindow outpost_window { util::deg2rad(70.0), util::deg2rad(30.0) }; // rad
    const double min_switch_improvement_angle { util::deg2rad(6.0) };

    double angular_velocity_threshold { 2 }; // rad/s

    int last_chosen_id { kNoArmor };

    auto should_switch_target(double current_abs_error, double candidate_abs_error) const noexcept
        -> bool {
        return candidate_abs_error + min_switch_improvement_angle < current_abs_error;
    }

    static auto abs_error(ArmorCandidate const& candidate) noexcept -> double {
        return std::abs(candidate.delta_yaw);
    }

    auto angle_window(DeviceId genre) const noexcept -> AngleWindow const& {
        return genre == DeviceId::OUTPOST ? outpost_window : normal_window;
    }

    auto choose_low_speed(
        std::span<ArmorCandidate const> candidates, AngleWindow const& window) const -> int {
        auto const in_window = [&](ArmorCandidate const& candidate) {
            return abs_error(candidate) < window.coming;
        };

        auto best_in_window = std::optional<ArmorCandidate> {};
        for (auto const& candidate : candidates) {
            if (!in_window(candidate)) continue;
            if (!best_in_window.has_value() || abs_error(candidate) < abs_error(*best_in_window)) {
                best_in_window = candidate;
            }
        }

        if (!best_in_window.has_value()) return kNoArmor;

        auto const has_last_candidate =
            (last_chosen_id >= 0) && (static_cast<size_t>(last_chosen_id) < candidates.size());
        if (!has_last_candidate) return best_in_window->index;

        auto const& last_candidate = candidates[static_cast<size_t>(last_chosen_id)];
        if (!in_window(last_candidate)) return best_in_window->index;
        if (best_in_window->index == last_chosen_id) return last_chosen_id;
        if (!should_switch_target(abs_error(last_candidate), abs_error(*best_in_window)))
            return last_chosen_id;

        return best_in_window->index;
    }

    auto choose_high_speed(
        std::span<ArmorCandidate const> candidates, AngleWindow const& window) const -> int {
        auto const is_in_window = [&](ArmorCandidate const& candidate) {
            return abs_error(candidate) < window.coming && candidate.phase <= window.leaving;
        };

        auto const is_incoming = [](ArmorCandidate const& candidate) {
            return candidate.phase < 0.0;
        };

        auto const is_better_than = [&](ArmorCandidate const& candidate,
                                        ArmorCandidate const& current_best) {
            auto const candidate_incoming = is_incoming(candidate);
            auto const best_incoming      = is_incoming(current_best);
            if (candidate_incoming != best_incoming) return candidate_incoming;

            return abs_error(candidate) < abs_error(current_best);
        };

        auto preferred = std::optional<ArmorCandidate> {};
        for (auto const& candidate : candidates) {
            if (!is_in_window(candidate)) continue;

            if (!preferred.has_value() || is_better_than(candidate, *preferred)) {
                preferred = candidate;
            }
        }

        if (!preferred) return kNoArmor;

        auto const has_last =
            (last_chosen_id >= 0) && (static_cast<size_t>(last_chosen_id) < candidates.size());
        if (!has_last) return preferred->index;

        auto const& last = candidates[static_cast<size_t>(last_chosen_id)];
        if (!is_in_window(last)) return preferred->index;
        if (preferred->index == last_chosen_id) return last_chosen_id;

        return should_switch_target(abs_error(last), abs_error(*preferred)) ? preferred->index
                                                                            : last_chosen_id;
    }

    auto initialize(Config const& config) noexcept -> void {
        normal_window  = { config.coming_angle, config.leaving_angle };
        outpost_window = { config.outpost_coming_angle, config.outpost_leaving_angle };

        angular_velocity_threshold = config.angular_velocity_threshold;
    }

    auto choose_armor(std::span<Armor3D const> armors, Eigen::Vector3d const& center_position,
        double angular_velocity) -> std::optional<Armor3D> {
        if (armors.empty()) {
            last_chosen_id = kNoArmor;
            return std::nullopt;
        }

        const auto center_yaw = std::atan2(center_position.y(), center_position.x());
        auto candidates       = std::array<ArmorCandidate, 4> {};
        const auto n          = std::min(armors.size(), candidates.size());

        for (size_t id = 0; id < n; ++id) {
            auto orientation = Eigen::Quaterniond {};
            armors[id].orientation.copy_to(orientation);

            const auto ypr          = util::eulers(orientation);
            const auto yaw_in_world = ypr[0];
            const auto delta_yaw    = util::normalize_angle(yaw_in_world - center_yaw);
            const auto phase        = (angular_velocity > 0.0) ? delta_yaw
                       : (angular_velocity < 0.0)              ? -delta_yaw
                                                               : 0.0;

            candidates[id] = { static_cast<int>(id), delta_yaw, phase };
        }

        auto chosen_id      = kNoArmor;
        auto candidate_view = std::span<ArmorCandidate const> { candidates }.first(n);
        auto const genre    = armors.front().genre;
        auto const& window  = angle_window(genre);

        // ---  非小陀螺模式 (低速旋转) ---
        if ((std::abs(angular_velocity) < angular_velocity_threshold)
            && (genre != DeviceId::OUTPOST)) {
            chosen_id = choose_low_speed(candidate_view, window);
        }
        // --- 小陀螺模式 (快速旋转) ---
        else {
            chosen_id = choose_high_speed(candidate_view, window);
        }

        if (chosen_id != kNoArmor) {
            last_chosen_id = chosen_id;
            return { armors[chosen_id] };
        }

        last_chosen_id = kNoArmor;
        return std::nullopt;
    }
};

AimPointChooser::AimPointChooser() noexcept
    : pimpl { std::make_unique<Impl>() } { }

AimPointChooser::~AimPointChooser() noexcept = default;

auto AimPointChooser::initialize(Config const& config) noexcept -> void {
    return pimpl->initialize(config);
}

auto AimPointChooser::choose_armor(std::span<Armor3D const> armors,
    Eigen::Vector3d const& center_position, double angular_velocity) -> std::optional<Armor3D> {
    return pimpl->choose_armor(armors, center_position, angular_velocity);
}
