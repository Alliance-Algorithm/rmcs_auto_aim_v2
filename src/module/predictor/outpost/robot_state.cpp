#include "robot_state.hpp"

#include "module/predictor/outpost/snapshot.hpp"
#include "utility/math/mahalanobis.hpp"
#include "utility/time.hpp"

#include <cmath>
#include <limits>
#include <optional>
#include <span>

namespace rmcs::predictor {

struct OutpostRobotState::Impl {
    explicit Impl(TimePoint stamp) noexcept
        : time_stamp { stamp } { }

    auto initialize(Armor3d const& armor, TimePoint t) -> void {
        color      = armor_color2camp_color(armor.color);
        ekf        = EKF { OutpostEKFParameters::x(armor),
            OutpostEKFParameters::P_initial_dig().asDiagonal() };
        time_stamp = t;

        layout                  = OutpostArmorLayout {};
        layout.height_levels[0] = 0;
        last_match              = LastMatch { 0, 0 };
        update_count            = 0;
        initialized             = true;
    }

    auto predict(TimePoint t) -> void {
        if (t <= time_stamp) return;

        if (initialized) {
            auto dt = rmcs::util::delta_time(t, time_stamp);
            if (dt > reset_interval) {
                reset_runtime_state(t);
                return;
            }

            auto const dt_s = dt.count();
            ekf.predict(
                OutpostEKFParameters::f(dt_s),
                [dt_s](EKF::XVec const&) { return OutpostEKFParameters::F(dt_s); },
                OutpostEKFParameters::Q(dt_s));
        }

        time_stamp = t;
    }

    auto update(std::span<Armor3d const> armors) -> bool {
        if (armors.empty()) return false;

        if (!initialized) {
            initialize(armors.front(), time_stamp);
            return true;
        }

        auto best_match = select_best_match(armors);
        if (!best_match.has_value()) return false;

        apply_match(*best_match);
        return true;
    }

    // TODO:添加收敛条件
    auto is_converged() const -> bool {
        if (!initialized) return false;
        return true;
    }

    auto get_snapshot() const -> std::optional<Snapshot> {
        if (!initialized) return std::nullopt;
        return Snapshot { OutpostSnapshot { ekf.x, color, time_stamp, layout } };
    }

    auto distance() const -> double {
        return initialized ? std::sqrt(ekf.x[0] * ekf.x[0] + ekf.x[2] * ekf.x[2])
                           : std::numeric_limits<double>::infinity();
    }

private:
    struct MatchedArmor {
        Armor3d armor;
        int height_level;
        double height_offset;
        OutpostArmorLayout layout;
    };

    struct LastMatch {
        int armor_id;
        int height_level;
    };

    static auto next_armor_id(int armor_id, int direction) -> int {
        constexpr auto armor_count = OutpostEKFParameters::kOutpostArmorCount;
        return (armor_id + direction + armor_count) % armor_count;
    }

    auto visit_match_hypotheses(auto&& visit) const -> void {
        if (!last_match.has_value()) return;

        auto const visit_hypothesis = [&](int armor_id, int height_level) {
            auto next_layout  = layout;
            auto& known_level = next_layout.height_levels[static_cast<std::size_t>(armor_id)];
            if (known_level.has_value() && *known_level != height_level) return;

            known_level = height_level;
            visit(armor_id, height_level, OutpostEKFParameters::height_offset(height_level),
                next_layout);
        };

        auto const& last = *last_match;
        visit_hypothesis(last.armor_id, last.height_level);

        auto const visit_next_hypotheses = [&](int direction, int first_delta, int second_delta) {
            auto const armor_id = next_armor_id(last.armor_id, direction);
            visit_hypothesis(armor_id, last.height_level + first_delta);
            visit_hypothesis(armor_id, last.height_level + second_delta);
        };

        auto const omega = ekf.x[6];
        if (omega > omega_deadzone) {
            visit_next_hypotheses(-1, +1, -2);
            return;
        }
        if (omega < -omega_deadzone) {
            visit_next_hypotheses(+1, -1, +2);
            return;
        }

        visit_next_hypotheses(-1, +1, -2);
        visit_next_hypotheses(+1, -1, +2);
    }

    auto evaluate_match(OutpostEKFParameters::Measurement const& measurement, Armor3d const& armor,
        double height_offset) const -> std::optional<double> {
        auto const H           = OutpostEKFParameters::H(ekf.x, armor, height_offset);
        auto const z_hat       = OutpostEKFParameters::h(ekf.x, armor, height_offset);
        auto const innovation  = OutpostEKFParameters::z_subtract(measurement.z, z_hat);
        auto const S           = H * ekf.P() * H.transpose() + measurement.R;
        auto const mahalanobis = rmcs::util::mahalanobis_distance(innovation, S);
        if (!mahalanobis.has_value() || *mahalanobis > mahalanobis_gate) return std::nullopt;

        return *mahalanobis;
    }

    auto reset_runtime_state(TimePoint t) -> void {
        color        = CampColor::UNKNOWN;
        ekf          = EKF {};
        layout       = OutpostArmorLayout {};
        last_match   = std::nullopt;
        time_stamp   = t;
        initialized  = false;
        update_count = 0;
    }

    auto select_best_match(std::span<Armor3d const> armors) const -> std::optional<MatchedArmor> {
        auto best_match       = std::optional<MatchedArmor> {};
        auto best_mahalanobis = std::numeric_limits<double>::infinity();

        for (auto const& armor : armors) {
            auto const measurement = OutpostEKFParameters::measurement(armor);

            visit_match_hypotheses([&](int armor_id, int height_level, double height_offset,
                                       OutpostArmorLayout const& next_layout) {
                auto matched_armor = armor;
                matched_armor.id   = armor_id;

                auto const mahalanobis = evaluate_match(measurement, matched_armor, height_offset);
                if (!mahalanobis.has_value()) return;
                if (*mahalanobis >= best_mahalanobis) return;

                best_mahalanobis = *mahalanobis;
                best_match =
                    MatchedArmor { matched_armor, height_level, height_offset, next_layout };
            });
        }

        return best_match;
    }

    auto apply_match(MatchedArmor const& match) -> void {
        auto const measurement = OutpostEKFParameters::measurement(match.armor);

        ekf.update(
            measurement.z,
            [armor = match.armor, height_offset = match.height_offset](
                EKF::XVec const& x) { return OutpostEKFParameters::h(x, armor, height_offset); },
            [armor = match.armor, height_offset = match.height_offset](
                EKF::XVec const& x) { return OutpostEKFParameters::H(x, armor, height_offset); },
            measurement.R, OutpostEKFParameters::x_add, OutpostEKFParameters::z_subtract);

        layout     = match.layout;
        last_match = LastMatch { match.armor.id, match.height_level };
        update_count++;
    }

    CampColor color { CampColor::UNKNOWN };
    EKF ekf { EKF {} };
    OutpostArmorLayout layout {};
    std::optional<LastMatch> last_match;
    TimePoint time_stamp;

    bool initialized { false };
    int update_count { 0 };
    std::chrono::duration<double> reset_interval { 1.5 };
    double mahalanobis_gate { 5.0 };
    double omega_deadzone { 0.2 };
};

OutpostRobotState::OutpostRobotState() noexcept
    : OutpostRobotState(Clock::now()) { }

OutpostRobotState::OutpostRobotState(TimePoint stamp) noexcept
    : pimpl { std::make_unique<Impl>(stamp) } { }

OutpostRobotState::~OutpostRobotState() noexcept = default;

auto OutpostRobotState::initialize(Armor3d const& armor, TimePoint t) -> void {
    return pimpl->initialize(armor, t);
}

auto OutpostRobotState::predict(TimePoint t) -> void { return pimpl->predict(t); }

auto OutpostRobotState::update(std::span<Armor3d const> armors) -> bool {
    return pimpl->update(armors);
}

auto OutpostRobotState::is_converged() const -> bool { return pimpl->is_converged(); }

auto OutpostRobotState::get_snapshot() const -> std::optional<Snapshot> {
    return pimpl->get_snapshot();
}

auto OutpostRobotState::distance() const -> double { return pimpl->distance(); }

} // namespace rmcs::predictor
